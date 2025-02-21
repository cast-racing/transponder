
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

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "iac_msgs/msg/competitor_estimate.hpp"

#include "iac_udp_struct.h"

class transponder2ros : public rclcpp::Node
{
public:
    transponder2ros();

private:

    // Subscribers / Publishers / Timers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavSatFix_;

    rclcpp::Publisher<iac_msgs::msg::CompetitorEstimate>::SharedPtr pubCompetitorEsimate_;

    rclcpp::TimerBase::SharedPtr timer_debug_;

    // Variables
    sockaddr_in receiver_addr_;
    int sockfd_ = socket(AF_INET,SOCK_DGRAM,0);

    std::mutex lock_;

    uint8_t car_id_ = 8;

    TransponderUdpPacket udp_out_;


    // Functions
    void init_udp();
    void init_ros();

    void push_udp();
    void push_ros();

    void callback_NavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void callback_debug();
  
};
