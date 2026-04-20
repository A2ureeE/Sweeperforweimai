#include "mc/can_utils.hpp"

#include <iomanip>
#include <sstream>

#include "logger/logger.h"

bool g_can_print_enable = false;

void receiveHandler(const CANFrame& frame, void* userData)
{
    auto* context = static_cast<CanHandlerContext*>(userData);
    auto node = context->node;
    auto pub = context->publisher;
    auto now = node->now();

    sweeper_interfaces::msg::CanFrame msg;
    msg.id = frame.id;
    msg.dlc = frame.dlc;
    msg.data.assign(frame.data, frame.data + frame.dlc);
    pub->publish(msg);

    if (g_can_print_enable)
    {
        std::stringstream ss;
        ss << "CAN ID: " << std::hex << std::uppercase << std::setw((frame.id > 0x7FF) ? 8 : 5) << std::setfill(' ')
           << frame.id << " Data: ";
        for (int i = 0; i < frame.dlc; ++i)
            ss << std::setw(2) << std::setfill('0') << std::hex << (int)frame.data[i] << " ";
        LOG_INFO("%s", ss.str().c_str());
    }
}