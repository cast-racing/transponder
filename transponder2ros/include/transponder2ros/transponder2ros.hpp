
#pragma once

#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <functional>
#include <mutex>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "iac_msgs/msg/competitor_estimate.hpp"

#include "iac_udp_struct.h"

class transponder2ros : public rclcpp::Node
{
public:
    transponder2ros();
    ~transponder2ros();

private:

    // Subscribers / Publishers / Timers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavSatFix_;

    rclcpp::Publisher<iac_msgs::msg::CompetitorEstimate>::SharedPtr pubCompetitorEsimate_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubVersion_;

    rclcpp::TimerBase::SharedPtr timer_1Hz_, timer_10s_;
    rclcpp::TimerBase::SharedPtr timer_debug_;

    // Threads
    TransponderUdpPacket udp_out_;

    // Variables
    sockaddr_in receiver_addr_;
    char buffer_[1024];
    struct sockaddr_in client_addr_;
    socklen_t addr_len_ = sizeof(client_addr_);
    int socket_fd;

    rclcpp::Time t_last_flag_ = this->get_clock()->now();
    double t_Udp_timeout_; // Switch to a parameter tonight
    bool has_timeout_ = false;

    int sockfd_ = socket(AF_INET,SOCK_DGRAM,0);

    std::mutex lock_;

    uint8_t car_id_ = 8;

    std::thread receive_data_thread_;


    // Functions
    void init_udp();
    void init_ros();

    void push_udp();
    void push_ros();

    void read_udpData();

    void callback_NavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    
    void callback_1Hz();
    void callback_10s();
    void callback_debug();
  
};
