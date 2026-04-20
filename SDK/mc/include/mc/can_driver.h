#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

struct CANFrame
{
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    bool ext; // 扩展帧
    bool rtr; // 远程帧
};

class CANDriver
{
public:
    using ReceiveCallback = std::function<void(const CANFrame &, void *)>;

    CANDriver();
    ~CANDriver();

    // 打开CAN接口
    bool open(const std::string &interface);

    // 关闭CAN接口
    void close();

    // 发送CAN帧
    bool sendFrame(const CANFrame &frame);

    // 设置接收回调
    void setReceiveCallback(ReceiveCallback callback, void *userData = nullptr);

    // 设置硬件过滤规则
    bool setFilter(const std::vector<can_filter> &filters);

    // 追加一个过滤器
    bool addFilter(const can_filter &filter);

    // 追加一组过滤器
    bool addFilters(const std::vector<can_filter> &filters);

private:
    void receiveThreadFunc();
    bool applyFilters(); // 应用当前filters_

    int sockfd = -1;
    std::atomic<bool> running{false};
    std::thread receiveThread;
    ReceiveCallback callback;
    void *userData = nullptr;
    std::vector<can_filter> filters_; // 当前所有过滤器
};

#endif // CAN_DRIVER_H
