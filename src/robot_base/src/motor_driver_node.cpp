#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial_driver/serial_port.hpp>
#include <memory>

using namespace drivers;

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode() : Node("motor_driver_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 9600);
        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);

        // Setup serial port configuration
        drivers::serial_driver::SerialPortConfig config;
        config.device_path = port_;
        config.baud_rate = baudrate_;
        config.flow_control = drivers::serial_driver::FlowControl::NONE;
        config.parity = drivers::serial_driver::Parity::NONE;
        config.stop_bits = drivers::serial_driver::StopBits::ONE;
        config.data_bits = drivers::serial_driver::CharSize::EIGHT;

        serial_port_ = std::make_unique<serial_driver::SerialPort>(
            this->get_logger(), this->get_clock());

        try {
            serial_port_->open(config);
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", port_.c_str(), baudrate_);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }

        // Subscribe to /cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        char command = 's';  // default stop
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        if (linear > 0.1)
            command = 'f';  // forward
        else if (linear < -0.1)
            command = 'b';  // backward
        else if (angular > 0.1)
            command = 'l';  // left
        else if (angular < -0.1)
            command = 'r';  // right

        std::string cmd_str(1, command);
        serial_port_->write(cmd_str);
        RCLCPP_INFO(this->get_logger(), "Sent command '%c' over serial", command);
    }

    std::string port_;
    int baudrate_;
    std::unique_ptr<serial_driver::SerialPort> serial_port_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriverNode>());
    rclcpp::shutdown();
    return 0;
}
