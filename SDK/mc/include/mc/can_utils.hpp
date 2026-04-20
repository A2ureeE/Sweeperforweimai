#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sweeper_interfaces/msg/can_frame.hpp"
#include "mc/can_struct.h"

extern bool g_can_print_enable;

struct CanHandlerContext
{
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<rclcpp::Publisher<sweeper_interfaces::msg::CanFrame>> publisher;
};

void receiveHandler(const CANFrame &frame, void *userData);
