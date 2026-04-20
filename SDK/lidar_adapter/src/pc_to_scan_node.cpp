#include <cmath>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer_interface.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>


class PcToScanNode : public rclcpp::Node
{
public:
    PcToScanNode()
    : Node("pc_to_scan_node")
    , tf_buffer_(this->get_clock())
    , tf_listener_(tf_buffer_)
    {
        this->declare_parameter("scan_frame", "base_footprint");
        this->declare_parameter("cloud_frame", "base_link");
        this->declare_parameter("input_topic", "/rslidar_points");
        this->declare_parameter("output_topic", "/scan");
        this->declare_parameter("angle_min", -3.14159);
        this->declare_parameter("angle_max", 3.14159);
        this->declare_parameter("angle_increment", 0.005);
        this->declare_parameter("range_min", 0.05);
        this->declare_parameter("range_max", 30.0);
        this->declare_parameter("height_tolerance", 0.5);
        this->declare_parameter("laser_z", 0.6);

        this->get_parameter("scan_frame", scan_frame_);
        this->get_parameter("cloud_frame", cloud_frame_);
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("angle_min", angle_min_);
        this->get_parameter("angle_max", angle_max_);
        this->get_parameter("angle_increment", angle_increment_);
        this->get_parameter("range_min", range_min_);
        this->get_parameter("range_max", range_max_);
        this->get_parameter("height_tolerance", height_tolerance_);
        this->get_parameter("laser_z", laser_z_);

        int num_values = static_cast<int>(
            std::ceil((angle_max_ - angle_min_) / angle_increment_)) + 1;

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 5,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->cb_cloud(msg);
            });

        RCLCPP_INFO(this->get_logger(),
                    "pc_to_scan_node ready: %s -> %s (frame=%s, laser_z=%.2fm)",
                    input_topic_.c_str(), output_topic_.c_str(),
                    scan_frame_.c_str(), laser_z_);
    }

private:
    void cb_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        if (cloud_msg->data.empty()) return;

        geometry_msgs::msg::TransformStamped tf_stamped;
        try
        {
            tf_stamped = tf_buffer_.lookupTransform(
                scan_frame_, cloud_msg->header.frame_id,
                rclcpp::Time(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec),
                rclcpp::Duration::from_seconds(0.5));
        }
        catch (const std::exception& e)
        {
            return;
        }

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = cloud_msg->header.stamp;
        scan.header.frame_id = scan_frame_;
        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_increment_;
        scan.time_increment = 0.0;
        scan.scan_time = 0.1;
        scan.range_min = range_min_;
        scan.range_max = range_max_;

        int num_values = static_cast<int>(
            std::ceil((angle_max_ - angle_min_) / angle_increment_)) + 1;
        scan.ranges.assign(num_values, std::numeric_limits<float>::infinity());

        double qx = tf_stamped.transform.rotation.x;
        double qy = tf_stamped.transform.rotation.y;
        double qz = tf_stamped.transform.rotation.z;
        double qw = tf_stamped.transform.rotation.w;
        double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz));
        double tx = tf_stamped.transform.translation.x;
        double ty = tf_stamped.transform.translation.y;
        double tz = tf_stamped.transform.translation.z;

        bool has_x = false, has_y = false, has_z = false, has_i = false;
        for (const auto& f : cloud_msg->fields)
        {
            if (f.name == "x") has_x = true;
            else if (f.name == "y") has_y = true;
            else if (f.name == "z") has_z = true;
            else if (f.name == "intensity") has_i = true;
        }

        if (!has_x || !has_y) return;

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float px = *iter_x;
            float py = *iter_y;
            float pz = (has_z ? *iter_z : 0.0f);

            if (!std::isfinite(px) || !std::isfinite(py)) continue;

            if (has_z && std::abs(pz - laser_z_) > height_tolerance_) continue;

            double rx = px + tx;
            double ry = py + ty;

            double angle = std::atan2(ry, rx) - yaw;
            while (angle > angle_max_) angle -= 2.0 * M_PI;
            while (angle < angle_min_) angle += 2.0 * M_PI;

            int idx = static_cast<int>(std::round((angle - angle_min_) / angle_increment_));
            if (idx < 0 || idx >= num_values) continue;

            double range = std::sqrt(rx * rx + ry * ry);
            if (range < range_min_ || range > range_max_) continue;

            if (range < scan.ranges[idx])
            {
                scan.ranges[idx] = static_cast<float>(range);
            }
        }

        pub_->publish(scan);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string scan_frame_;
    std::string cloud_frame_;
    std::string input_topic_;
    std::string output_topic_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;
    double height_tolerance_;
    double laser_z_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcToScanNode>());
    rclcpp::shutdown();
    return 0;
}
