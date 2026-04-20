#include <stdlib.h>
#include <string.h>

#include <queue>

#include "logger/logger.h"
#include "rclcpp/rclcpp.hpp"
#include "serial_read.hpp"
#include "sweeper_interfaces/msg/rtk.hpp"

class rtk_node : public rclcpp::Node
{
   public:
    // 构造函数,有一个参数为节点名称
    rtk_node(std::string name) : Node(name)
    {
        LOG_INFO("%s节点已经启动.", name.c_str());

        lat = 0.0;
        lon = 0.0;
        head = 0.0;
        speed = 0.0;

        // 初始化串口读取
        boost_serial = new Boost_serial();
        std::string serial_port;
        this->declare_parameter("serial_port", "/dev/ttyTHS1");
        this->get_parameter("serial_port", serial_port);
        boost_serial->init(serial_port.c_str());

        // 创建发布者
        command_publisher_ = this->create_publisher<sweeper_interfaces::msg::Rtk>("rtk_message", 10);
        // 创建定时器，100ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&rtk_node::timer_callback, this));
    }

   private:
    void GPYBM_to_gps(char serial_buf[])
    {
        position = 0;
        line_string.clear();

        char* p_gpybm = strstr(serial_buf, "GPYBM");

        if (p_gpybm == NULL)
        {
            LOG_INFO("未检测到GPYBM字符串!");
            return;
        }

        line_string = p_gpybm;

        // 解析过程添加完整的错误检查
        try
        {
            // 跳过前3个字段
            for (int i = 0; i < 3; i++)
            {
                position = line_string.find(",");
                if (position == string::npos)
                {
                    LOG_WARN("GPYBM数据格式错误：字段数量不足");
                    return;
                }
                line_string.erase(0, position + 1);
            }

            // 解析纬度
            position = line_string.find(",");
            if (position == string::npos)
            {
                LOG_WARN("GPYBM数据格式错误：缺少纬度字段");
                return;
            }
            lat_s = line_string.substr(0, position);
            line_string.erase(0, position + 1);

            // 解析经度
            position = line_string.find(",");
            if (position == string::npos)
            {
                LOG_WARN("GPYBM数据格式错误：缺少经度字段");
                return;
            }
            lon_s = line_string.substr(0, position);
            line_string.erase(0, position + 1);

            // 跳过1个字段
            position = line_string.find(",");
            if (position == string::npos)
            {
                LOG_WARN("GPYBM数据格式错误：字段数量不足");
                return;
            }
            line_string.erase(0, position + 1);

            // 解析航向
            position = line_string.find(",");
            if (position == string::npos)
            {
                LOG_WARN("GPYBM数据格式错误：缺少航向字段");
                return;
            }
            head_s = line_string.substr(0, position);
            line_string.erase(0, position + 1);

            // 跳过4个字段
            for (int i = 0; i < 4; i++)
            {
                position = line_string.find(",");
                if (position == string::npos)
                {
                    LOG_WARN("GPYBM数据格式错误：字段数量不足");
                    return;
                }
                line_string.erase(0, position + 1);
            }

            // 解析速度
            position = line_string.find(",");
            if (position == string::npos)
            {
                LOG_WARN("GPYBM数据格式错误：缺少速度字段");
                return;
            }
            sp_s = line_string.substr(0, position);
            line_string.erase(0, position + 1);

            // 跳过4个字段
            for (int i = 0; i < 4; i++)
            {
                position = line_string.find(",");
                if (position == string::npos)
                {
                    LOG_WARN("GPYBM数据格式错误：字段数量不足");
                    return;
                }
                line_string.erase(0, position + 1);
            }

            // 解析定位质量
            position = line_string.find(",");
            if (position == string::npos)
            {
                LOG_WARN("GPYBM数据格式错误：缺少定位质量字段");
                return;
            }
            p_q_s = line_string.substr(0, position);
            line_string.erase(0, position + 1);

            // 解析定向质量
            position = line_string.find(",");
            if (position == string::npos)
            {
                LOG_WARN("GPYBM数据格式错误：缺少定向质量字段");
                return;
            }
            h_q_s = line_string.substr(0, position);
            line_string.erase(0, position + 1);

            // 验证字段内容是否有效
            if (lat_s.empty() || lon_s.empty() || head_s.empty() || sp_s.empty() || p_q_s.empty() || h_q_s.empty())
            {
                LOG_WARN("GPYBM数据格式错误：字段内容为空");
                return;
            }

            // 检查字段是否包含合法的数字字符
            auto is_valid_number = [](const string& s) -> bool
            {
                if (s.empty()) return false;
                for (char c : s)
                {
                    if (!isdigit(c) && c != '.' && c != '-' && c != '+')
                    {
                        return false;
                    }
                }
                return true;
            };

            if (!is_valid_number(lat_s) || !is_valid_number(lon_s) || !is_valid_number(head_s) ||
                !is_valid_number(sp_s) || !is_valid_number(p_q_s) || !is_valid_number(h_q_s))
            {
                LOG_WARN("GPYBM数据格式错误：字段包含非法字符");
                return;
            }

            // 转换为数值
            lat = atof(lat_s.c_str());
            lon = atof(lon_s.c_str());
            head = atof(head_s.c_str());
            speed = atof(sp_s.c_str());
            p_quality = atof(p_q_s.c_str());
            h_quality = atof(h_q_s.c_str());

            // 验证地理坐标范围（简单验证）
            if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
            {
                LOG_WARN("GPYBM数据错误：坐标值超出范围");
                lat = 0.0;
                lon = 0.0;
                return;
            }
        }
        catch (const exception& e)
        {
            LOG_ERROR("解析GPYBM数据时发生异常：%s", e.what());
            return;
        }
        catch (...)
        {
            LOG_ERROR("解析GPYBM数据时发生未知异常");
            return;
        }
    }

    void timer_callback()
    {
        try
        {
            // 读取串口传来的定位信息
            memset(serial_buf, 0, sizeof(serial_buf));
            int num = boost_serial->serial_read(serial_buf, 200);

            if (num < 0)
            {
                LOG_WARN("串口读取失败：返回值为负");
                return;
            }

            if (c_queue.size() >= 400)
            {
                std::queue<char> empty;
                swap(empty, c_queue);
                LOG_WARN("队列已满，已清空队列");
            }

            for (int i = 0; i < num; i++)
            {
                if (serial_buf[i] == '*')
                {
                    memset(gps_buf, 0, sizeof(gps_buf));
                    size_t j = 0;  // 改为 size_t 类型，避免与 sizeof 结果比较的警告
                    while (!c_queue.empty() && j < sizeof(gps_buf) - 1)
                    {
                        gps_buf[j] = c_queue.front();
                        c_queue.pop();
                        j++;
                    }
                    gps_buf[j] = '\0';  // 确保字符串结束符

                    // 检查缓冲区是否足够大
                    if (j >= sizeof(gps_buf) - 1)
                    {
                        LOG_WARN("GPS数据过长，可能导致缓冲区溢出");
                        std::queue<char> empty;
                        swap(empty, c_queue);
                        continue;
                    }

                    // 解析定位信息
                    GPYBM_to_gps(gps_buf);

                    // 创建消息
                    auto message = sweeper_interfaces::msg::Rtk();

                    message.lat = lat;
                    message.lon = lon;
                    message.head = head;
                    message.speed = speed;
                    message.p_quality = p_quality;
                    message.h_quality = h_quality;

                    // 日志打印
                    LOG_INFO("lat:'%.9lf',lon:'%.9lf',head:'%lf',speed:'%lf',p_quality:'%d',h_quality:'%d'",
                             message.lat, message.lon, message.head, message.speed, message.p_quality,
                             message.h_quality);
                    // 发布消息
                    command_publisher_->publish(message);
                }
                else
                {
                    c_queue.push(serial_buf[i]);
                }
            }
        }
        catch (const exception& e)
        {
            LOG_ERROR("定时器回调函数异常：%s", e.what());
        }
        catch (...)
        {
            LOG_ERROR("定时器回调函数未知异常");
        }
    }

    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<sweeper_interfaces::msg::Rtk>::SharedPtr command_publisher_;

    // 串口读取类指针
    Boost_serial* boost_serial;

    // 串口读取buffer
    char serial_buf[300];
    char gps_buf[300];

    double lat;
    double lon;
    double head;
    double speed;
    int p_quality;  // 定位解状态：0=未定位或无效解 1=单点定位 4=定位RTK固定解 5=定位RTK浮点解
    int h_quality;  // 定向解状态：0=未定位或无效解 1=单点定位 4=定位RTK固定解 5=定位RTK浮点解

    std::queue<char> c_queue;
    // 解析定位信息用到的中间变量
    string lat_s, lon_s, head_s, sp_s, p_q_s, h_q_s;
    size_t position;  // 改为 size_t 类型，匹配 string::find() 的返回类型
    string line_string;
};

int main(int argc, char** argv)
{
    // 初始化日志系统
    logger::Logger::Init("rtk", "./nodes_log");

    // unsigned int a = -1;
    // LOG_DEBUG("%u\n", a);
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<rtk_node>("rtk_node");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();

    // 关闭日志系统
    logger::Logger::Shutdown();
    return 0;
}
