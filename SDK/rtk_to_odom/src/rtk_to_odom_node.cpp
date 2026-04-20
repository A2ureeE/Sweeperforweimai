#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <sweeper_bridge_interfaces/msg/rtk.hpp>

class RtkToOdomNode : public rclcpp::Node
{
public:
    RtkToOdomNode()
    : Node("rtk_to_odom_node"), origin_set_(false)
    {
        this->declare_parameter("ref_lat", 0.0);
        this->declare_parameter("ref_lon", 0.0);
        this->declare_parameter("use_rtk_heading", true);
        this->declare_parameter("quality_thresh", 4);
        this->declare_parameter("broadcast_tf", true);
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_footprint");

        this->get_parameter("ref_lat", ref_lat_);
        this->get_parameter("ref_lon", ref_lon_);
        this->get_parameter("use_rtk_heading", use_rtk_heading_);
        this->get_parameter("quality_thresh", quality_thresh_);
        this->get_parameter("broadcast_tf", broadcast_tf_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_frame", base_frame_);

        if (ref_lat_ != 0.0 && ref_lon_ != 0.0)
        {
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Using configured origin: lat=%.6f, lon=%.6f",
                        ref_lat_, ref_lon_);
        }

        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = this->create_subscription<sweeper_bridge_interfaces::msg::Rtk>(
            "/rtk_message", 10,
            [this](const sweeper_bridge_interfaces::msg::Rtk::SharedPtr msg) { this->cb_rtk(msg); });

        RCLCPP_INFO(this->get_logger(), "rtk_to_odom_node ready");
    }

private:
    static double wrapAngle(double a)
    {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    void cb_rtk(const sweeper_bridge_interfaces::msg::Rtk::SharedPtr msg)
    {
        if (msg->p_quality < quality_thresh_)
        {
            return;
        }

        double lat = msg->lat;
        double lon = msg->lon;
        double heading = msg->head * M_PI / 180.0;

        if (!origin_set_)
        {
            ref_lat_ = lat;
            ref_lon_ = lon;
            origin_set_ = true;
            last_lat_ = lat;
            last_lon_ = lon;
            RCLCPP_INFO(this->get_logger(),
                        "RTK origin established: lat=%.6f, lon=%.6f",
                        ref_lat_, ref_lon_);
            return;
        }

        double x_local = (lon - ref_lon_) * DEG_TO_M_LON_ * std::cos(ref_lat_ * M_PI / 180.0);
        double y_local = (lat - ref_lat_) * DEG_TO_M_LAT;

        last_lat_ = lat;
        last_lon_ = lon;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = x_local;
        odom.pose.pose.position.y = y_local;
        odom.pose.pose.position.z = 0.0;

        double yaw = use_rtk_heading_ ? heading : last_heading_;
        if (use_rtk_heading_)
        {
            double diff = wrapAngle(yaw - last_heading_);
            last_heading_ += diff * 0.3;
            last_heading_ = wrapAngle(last_heading_);
        }
        else
        {
            last_heading_ = yaw;
        }

        double qx = std::sin(last_heading_ * 0.5);
        double qy = 0.0;
        double qz = 0.0;
        double qw = std::cos(last_heading_ * 0.5);
        odom.pose.pose.orientation.x = qx;
        odom.pose.pose.orientation.y = qy;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;

        odom.twist.twist.linear.x = msg->speed;
        odom.twist.twist.angular.z = 0.0;

        pub_odom_->publish(odom);

        if (broadcast_tf_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = odom.header.stamp;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = x_local;
            tf.transform.translation.y = y_local;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    static constexpr double DEG_TO_M_LAT = 111320.0;
    static constexpr double DEG_TO_M_LON = 111320.0;

    rclcpp::Subscription<sweeper_bridge_interfaces::msg::Rtk>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool origin_set_;
    double ref_lat_ = 0.0;
    double ref_lon_ = 0.0;
    double last_lat_ = 0.0;
    double last_lon_ = 0.0;
    double last_heading_ = 0.0;
    bool use_rtk_heading_;
    int quality_thresh_;
    bool broadcast_tf_;
    std::string odom_frame_;
    std::string base_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RtkToOdomNode>());
    rclcpp::shutdown();
    return 0;
}
