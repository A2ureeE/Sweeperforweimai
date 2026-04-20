#pragma once
#include "rclcpp/rclcpp.hpp"
#include "mc/can_driver.h"
#include <functional>

// 注册所有定时器任务
void setupTimers(rclcpp::Node::SharedPtr node, CANDriver &canctl);

// 注册 VID 发送定时器任务
void setupVidTimer(rclcpp::Node::SharedPtr node, std::function<void()> callback);
