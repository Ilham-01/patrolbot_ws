#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <serial_driver/serial_port.hpp>
#include <string>
#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;
using namespace drivers;

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 9600);
        this->declare_parameter<double>("wheel_radius", 0.033);
        this->declare_parameter<double>("wheel_base", 0.16);
        this->declare_parameter<int>("ticks_per_revolution", 3200);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        ticks_per_rev_ = this->get_parameter("ticks_per_revolution").as_int();

        // Configure and open the serial port
        serial_driver::SerialPortConfig config;
        config.device_path = port_;
        config.baud_rate = baudrate_;
        config.flow_control = serial_driver::FlowControl::NONE;
        config.parity = serial_driver::Parity::NONE;
        config.stop_bits = serial_driver::StopBits::ONE;
        config.data_bits = serial_driver::CharSize::EIGHT;

        serial_port_ = std::make_unique<serial_driver::SerialPort>(
            this->get_logger(), this->get_clock());

        try
        {
            serial_port_->open(config);
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened", port_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to open serial port %s: %s", port_.c_str(), e.what());
            rclcpp::shutdown();
        }

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(50ms, std::bind(&OdometryNode::update, this));
    }

private:
    void update()
    {
        std::string line;
        try {
            line = serial_port_->readline(100);  // Timeout: 100ms
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Serial read failed: %s", e.what());
            return;
        }

        int enc_left = 0, enc_right = 0;
        if (sscanf(line.c_str(), "ENC_L:%d ENC_R:%d", &enc_left, &enc_right) == 2)
        {
            auto now = this->get_clock()->now();

            double dl = (enc_left - prev_left_) * 2 * M_PI * wheel_radius_ / ticks_per_rev_;
            double dr = (enc_right - prev_right_) * 2 * M_PI * wheel_radius_ / ticks_per_rev_;

            prev_left_ = enc_left;
            prev_right_ = enc_right;

            double dc = (dr + dl) / 2.0;
            double dtheta = (dr - dl) / wheel_base_;

            x_ += dc * cos(theta_ + dtheta / 2.0);
            y_ += dc * sin(theta_ + dtheta / 2.0);
            theta_ += dtheta;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = now;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            odom_pub_->publish(odom);

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = "odom";
            tf.child_frame_id = "base_link";
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(tf);
        }
    }

    std::unique_ptr<serial_driver::SerialPort> serial_port_;
    std::string port_;
    int baudrate_;
    double wheel_radius_;
    double wheel_base_;
    int ticks_per_rev_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    int prev_left_ = 0, prev_right_ = 0;
    double x_, y_, theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
