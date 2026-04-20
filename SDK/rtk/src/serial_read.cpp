#include "serial_read.hpp"

#include "logger/logger.h"

Boost_serial::Boost_serial() : sp(nullptr) { ; }

Boost_serial::~Boost_serial()
{
    if (sp)
    {
        try
        {
            if (sp->is_open())
            {
                sp->close();
            }
            delete sp;
        }
        catch (...)
        {
            LOG_ERROR("关闭串口时发生异常");
        }
    }
}

void Boost_serial::init(const string port_name)
{
    try
    {
        sp = new serial_port(service);
        sp->open(port_name);

        if (sp->is_open())
        {
            sp->set_option(serial_port::baud_rate(115200));
            sp->set_option(serial_port::flow_control(serial_port::flow_control::none));
            sp->set_option(serial_port::parity(serial_port::parity::none));
            sp->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            sp->set_option(serial_port::character_size(8));
            LOG_INFO("打开串口成功！");
        }
        else
        {
            LOG_ERROR("打开串口失败！");
            delete sp;
            sp = nullptr;
        }
    }
    catch (const exception& e)
    {
        LOG_ERROR("初始化串口时发生异常：%s", e.what());
        if (sp)
        {
            delete sp;
            sp = nullptr;
        }
    }
    catch (...)
    {
        LOG_ERROR("初始化串口时发生未知异常");
        if (sp)
        {
            delete sp;
            sp = nullptr;
        }
    }
}

int Boost_serial::serial_read(char buf[], int size)
{
    if (sp == nullptr || !sp->is_open())
    {
        LOG_WARN("串口未初始化或未打开");
        return -1;
    }

    try
    {
        int num = read(*sp, buffer(buf, size));
        if (num < 0)
        {
            LOG_WARN("串口读取错误");
            return -1;
        }
        else if (num == 0)
        {
            LOG_INFO("串口无数据可读");
            return 0;
        }
        return num;
    }
    catch (const exception& e)
    {
        LOG_ERROR("串口读取异常：%s", e.what());
        return -1;
    }
    catch (...)
    {
        LOG_ERROR("串口读取未知异常");
        return -1;
    }
}
