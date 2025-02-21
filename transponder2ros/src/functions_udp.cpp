#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_udp()
{

    // Get the param
    this->declare_parameter("car_ip", "0.0.0.0");
    this->declare_parameter<int>("udp_port_receive", 15783);
    this->declare_parameter<int>("udp_port_send", 15782);

    std::string param_ip_address = this->get_parameter("car_ip").as_string();
    uint16_t param_port = this->get_parameter("udp_port_receive").as_int();

    RCLCPP_INFO(this->get_logger(), "\tStreaming transponder to %s:%d",param_ip_address.c_str(), param_port);

    // Create a UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "transponder failed to create socket");
        rclcpp::shutdown();
    }

    // Set up the receiver address
    bzero(&receiver_addr_,sizeof(receiver_addr_));
    receiver_addr_.sin_family = AF_INET;
    receiver_addr_.sin_addr.s_addr = inet_addr(param_ip_address.c_str());
    receiver_addr_.sin_port = htons(param_port);

    // All done
    return;
}

void transponder2ros::push_udp()
{

    // Send data
    ssize_t sent_bytes = sendto(
        sockfd_, udp_out_.raw, sizeof(udp_out_.raw), 0,
         (sockaddr*)&receiver_addr_, sizeof(receiver_addr_));

    if (sent_bytes < 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1UL * 1000 * 1000, "Failed to send telop UDP packet");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Sent %ld bytes", sent_bytes);
        RCLCPP_INFO(this->get_logger(), "Time %f", udp_out_.data.utc);
    }

    // Debugging
    if (0)
    {
        RCLCPP_INFO(this->get_logger(), "Sending %5.2f, %5.2f, %5.2f",
        udp_out_.data.lat, udp_out_.data.lon, udp_out_.data.vel);
    }

    // All done
    return;
}

