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

    // UDP socket listener
    receive_data_thread_ = std::thread(&transponder2ros::read_udpData, this);


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


void transponder2ros::read_udpData()
{

    // Listen to UDP port and convert data to ROS2 topics
    while (rclcpp::ok())
    {
        // Get available data
        ssize_t len = recvfrom(socket_fd, buffer_, sizeof(buffer_) - 1, 0, (struct sockaddr *)&client_addr_, &addr_len_);

        if (len > 0)
        {
            // RCLCPP_INFO(this->get_logger(), "Got %ld bytes", len);

            switch (len)
            {
            case (SIZEOF_TransponderUdpPacket):
            {

                // Teleop UDP Packet
                TransponderUdpPacket transponder;

                // std::lock_guard<std::mutex> lock(lock_);
                std::memcpy(&transponder.data, &buffer_, SIZEOF_TransponderUdpPacket);

                // Reject if message is too old
                double msg_tDiff = this->get_clock()->now().seconds() - transponder.data.utc;

                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Latency: %.3f s", msg_tDiff);

                if (abs(msg_tDiff) > t_Udp_timeout_)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "UDP transponder packet late by %.3f s", msg_tDiff);
                    continue;
                }

                // Print the data (debug)

                

                // Publish the data
                // teleop_msgs::msg::Teleop msg_Teleop;

                // msg_Teleop.header.stamp = this->get_clock()->now();
                // msg_Teleop.header.frame_id = "teleop (udp)";

                // msg_Teleop.throttle = msg.data.accelerator;
                // msg_Teleop.brake = msg.data.brake;
                // msg_Teleop.steering = msg.data.steering;
                // msg_Teleop.gear = msg.data.gear;
                // msg_Teleop.in_command.in_command = msg.data.command;
                // msg_Teleop.arm.state = msg.data.armed;
                // pubDrive_->publish(msg_Teleop);

                // Update our last received time
                t_last_flag_ = this->get_clock()->now();

                // Update user if reconnected
                if (has_timeout_)
                {
                    RCLCPP_INFO(this->get_logger(), "Transponder UDP connected");
                    has_timeout_ = false;
                }

                // Debugging
                if (0)
                {
                    RCLCPP_INFO(this->get_logger(), "lat: %8.3f, lon: %8.3f",
                        transponder.data.lat, transponder.data.lon
                    );
                }

                break;
            }
            default:
            {
                RCLCPP_INFO(this->get_logger(), "Unrecognised packet length of %ld bytes", len);
                break;
            }
            }
        }
    }
    // All done
    return;
}