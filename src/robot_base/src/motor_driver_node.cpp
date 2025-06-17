#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <cerrno>

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode() : Node("motor_driver_node"), serial_fd_(-1)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 9600);
        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);

        openSerialPort();

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1));
    }

    ~MotorDriverNode()
    {
        if (serial_fd_ != -1)
            close(serial_fd_);
    }

private:
    std::string port_;
    int baudrate_;
    int serial_fd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void openSerialPort()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port %s: %s", port_.c_str(), strerror(errno));
            rclcpp::shutdown();
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        cfsetospeed(&tty, baudrateToConstant(baudrate_));
        cfsetispeed(&tty, baudrateToConstant(baudrate_));

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 1;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", port_.c_str(), baudrate_);
        }
    }

    speed_t baudrateToConstant(int baudrate)
    {
        switch (baudrate)
        {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default: return B9600;
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (serial_fd_ == -1) return;

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

        int bytes_written = write(serial_fd_, &command, 1);
        if (bytes_written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent command '%c' over serial", command);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriverNode>());
    rclcpp::shutdown();
    return 0;
}
