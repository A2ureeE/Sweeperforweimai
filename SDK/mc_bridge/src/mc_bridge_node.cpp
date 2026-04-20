#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sweeper_interfaces/msg/mc_ctrl.hpp>

class McBridgeNode : public rclcpp::Node
{
public:
    McBridgeNode()
    : Node("mc_bridge_node")
    {
        this->declare_parameter("wheel_base", 1.05);
        this->declare_parameter("max_rpm", 6000.0);
        this->declare_parameter("max_speed", 1.3);
        this->declare_parameter("max_steer_angle", 0.8727);
        this->declare_parameter("steer_scale", 7.0);
        this->declare_parameter("steer_offset", 1024.0);
        this->declare_parameter("rate_hz", 20.0);

        this->get_parameter("wheel_base", wheel_base_);
        this->get_parameter("max_rpm", max_rpm_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("max_steer_angle", max_steer_angle_);
        this->get_parameter("steer_scale", steer_scale_);
        this->get_parameter("steer_offset", steer_offset_);
        double rate_hz;
        this->get_parameter("rate_hz", rate_hz);

        pub_ctrl_ = this->create_publisher<sweeper_interfaces::msg::McCtrl>("/mc_ctrl", 10);

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->cb_cmd(msg); });

        sub_mode_ = this->create_subscription<std_msgs::msg::String>(
            "/behavior/mode", 5,
            [this](const std_msgs::msg::String::SharedPtr msg) { this->mode_ = msg->data; });

        sub_speed_ = this->create_subscription<std_msgs::msg::Float32>(
            "/behavior/speed_limit", 5,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { this->speed_limit_ = msg->data; });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
            [this]() { this->tick(); });

        RCLCPP_INFO(this->get_logger(), "mc_bridge_node ready (wheel_base=%.2fm, max_rpm=%d)",
                    wheel_base_, static_cast<int>(max_rpm_));
    }

private:
    void cb_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        v_cmd_ = msg->linear.x;
        omega_cmd_ = msg->angular.z;
        new_cmd_ = true;
    }

    void tick()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        sweeper_interfaces::msg::McCtrl ctrl;

        if (!new_cmd_)
        {
            ctrl.brake = 1;
            ctrl.gear = 0;
            ctrl.rpm = 0;
            ctrl.angle = 0.0f;
            ctrl.angle_speed = 120;
            ctrl.sweep = false;
            ctrl.sweep_mode = 0;
            ctrl.enable_main_brush = false;
            ctrl.enable_vacuum = false;
            ctrl.enable_dust_shake = false;
            ctrl.enable_main_brush_pole = false;
            ctrl.enable_flap_pole = false;
            ctrl.enable_side_brush = false;
            ctrl.enable_water_pump = false;
            pub_ctrl_->publish(ctrl);
            return;
        }
        new_cmd_ = false;

        double v = std::abs(v_cmd_);
        bool forward = (v_cmd_ >= 0.0);

        if (mode_ == "STOP")
        {
            v = 0.0;
        }

        if (v < 0.01)
        {
            ctrl.brake = 1;
            ctrl.gear = 0;
            ctrl.rpm = 0;
        }
        else
        {
            ctrl.brake = 0;
            ctrl.gear = forward ? 2 : 1;

            double rpm_double = v * (max_rpm_ / max_speed_);
            rpm_double = std::clamp(rpm_double, 0.0, static_cast<double>(max_rpm_));
            ctrl.rpm = static_cast<uint16_t>(std::round(rpm_double));
        }

        if (std::abs(omega_cmd_) < 1e-4 || std::abs(v_cmd_) < 0.01)
        {
            ctrl.angle = 0.0f;
        }
        else
        {
            double angle_raw = std::atan(omega_cmd_ * wheel_base_ / v_cmd_);
            angle_raw = std::clamp(angle_raw, -max_steer_angle_, max_steer_angle_);
            ctrl.angle = static_cast<float>(angle_raw);
        }

        ctrl.angle_speed = 300;

        ctrl.sweep = true;
        ctrl.sweep_mode = 0;
        ctrl.enable_main_brush = true;
        ctrl.enable_vacuum = true;
        ctrl.enable_dust_shake = false;
        ctrl.enable_main_brush_pole = true;
        ctrl.enable_flap_pole = true;
        ctrl.enable_side_brush = true;
        ctrl.enable_water_pump = true;

        pub_ctrl_->publish(ctrl);
    }

    rclcpp::Publisher<sweeper_interfaces::msg::McCtrl>::SharedPtr pub_ctrl_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    double v_cmd_ = 0.0;
    double omega_cmd_ = 0.0;
    bool new_cmd_ = false;
    std::string mode_ = "COVERAGE";
    double speed_limit_ = 1.3;

    double wheel_base_ = 1.05;
    double max_rpm_ = 6000.0;
    double max_speed_ = 1.3;
    double max_steer_angle_ = 0.8727;
    double steer_scale_ = 7.0;
    double steer_offset_ = 1024.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<McBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
