#include <iostream>

#include "logger/logger.h"
#include "mc/can_driver.h"
#include "mc/can_utils.hpp"
#include "mc/control_cache.hpp"
#include "mc/get_config.h"
#include "mc/timer_tasks.hpp"
#include "mc/vid_transmit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sweeper_interfaces/msg/can_frame.hpp"
#include "sweeper_interfaces/msg/mc_ctrl.hpp"
#include "sweeper_interfaces/msg/vehicle_identity.hpp"

namespace sweeperMsg = sweeper_interfaces::msg;

CANDriver canctl;
std::string current_vid;

void vehicleIdentityCallback(const sweeperMsg::VehicleIdentity::SharedPtr msg)
{
    if (msg->ready && !msg->vid.empty() && msg->vid != current_vid)
    {
        LOG_INFO("Received new VID: %s", msg->vid.c_str());
        current_vid = msg->vid;
    }
}

void send_vid_task()
{
    if (!current_vid.empty())
    {
        if (!vid_transmit::send_vid_to_can(canctl, current_vid))
        {
            LOG_WARN("Failed to send VID, will retry");
        }
    }
}

void mcCtrlCallback(const sweeperMsg::McCtrl::SharedPtr msg)
{
    // LOG_INFO("\n  刹车: %s", (msg->brake ? "已刹车" : "未刹车"));
    // LOG_INFO("  挡位: ");
    // switch (msg->gear)
    // {
    // case 0:
    //   LOG_INFO("空挡");
    //   break;
    // case 2:
    //   LOG_INFO("前进挡");
    //   break;
    // case 1:
    //   LOG_INFO("后退挡");
    //   break;
    // default:
    //   LOG_INFO("未知挡位(%d)", static_cast<int>(msg->gear));
    //   break;
    // }
    // LOG_INFO("  行走电机转速: %d RPM", static_cast<int>(msg->rpm));
    // LOG_INFO("  轮端转向角度: %.1f°", msg->angle);
    // LOG_INFO("  清扫状态: %s", (msg->sweep ? "正在清扫" : "未清扫"));

    control_cache.update(*msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    /*初始化日志系统*/
    logger::Logger::Init("mc", "./nodes_log");

    auto node = rclcpp::Node::make_shared("can_driver_node");
    LOG_INFO("Starting mc package...");

    auto pub = node->create_publisher<sweeperMsg::CanFrame>("can_data", 10);
    auto sub = node->create_subscription<sweeperMsg::McCtrl>("mc_ctrl", 10, mcCtrlCallback);
    auto vid_sub = node->create_subscription<sweeperMsg::VehicleIdentity>("/vehicle/identity", rclcpp::QoS(1).transient_local().reliable(), vehicleIdentityCallback);

    Config mc_config;
    load_config(mc_config);

    if (!canctl.open(mc_config.can_dev))
    {
        LOG_ERROR("Failed to open CAN interface: %s", mc_config.can_dev.c_str());
        logger::Logger::Shutdown();
        return -1;
    }

    auto context = std::make_shared<CanHandlerContext>();
    context->node = node;
    context->publisher = pub;
    canctl.setReceiveCallback(receiveHandler, context.get());

    setupTimers(node, canctl);
    setupVidTimer(node, send_vid_task);

    rclcpp::on_shutdown([&]() { canctl.close(); });
    rclcpp::spin(node);
    rclcpp::shutdown();
    /*关闭日志系统*/
    logger::Logger::Shutdown();
    return 0;
}
