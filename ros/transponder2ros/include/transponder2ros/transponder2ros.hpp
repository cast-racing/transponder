
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
#include "transponder_msgs/msg/transponder.hpp"

#include "iac_udp_struct.h"

class transponder2ros : public rclcpp::Node
{
public:
    transponder2ros();
    ~transponder2ros();

private:

    // Subscribers / Publishers / Timers
    rclcpp::Subscription<transponder_msgs::msg::Transponder>::SharedPtr sub_Transponder_;

    rclcpp::Publisher<transponder_msgs::msg::Transponder>::SharedPtr pub_Transponder_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_Version_;

    rclcpp::TimerBase::SharedPtr timer_1Hz_, timer_10s_;
    rclcpp::TimerBase::SharedPtr timer_debug_;

    // Threads
    std::thread receive_data_thread_;

    // Variables
    sockaddr_in send_addr_;

    char buffer_[1024];
    struct sockaddr_in client_addr_;
    socklen_t addr_len_ = sizeof(client_addr_);
    int read_addr_; //socket_fd;

    rclcpp::Time t_last_flag_ = this->get_clock()->now();
    double t_Udp_timeout_; // Switch to a parameter tonight
    bool has_timeout_ = false;

    int sockfd_ = socket(AF_INET,SOCK_DGRAM,0);

    std::mutex lock_;

    uint8_t car_id_ = 8;


    // Functions
    void init_udp();
    void init_ros();

    void push_udp(StructIacTransponder data);
    // void push_ros();

    void read_udpData();

    void callback_Transponder(const transponder_msgs::msg::Transponder::SharedPtr msg);
    
    void callback_1Hz();
    void callback_10s();
    void callback_debug();
  
};
