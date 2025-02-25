#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_udp()
{

    // Get the param
    this->declare_parameter("ros_ip", "127.0.0.1");
    this->declare_parameter<int>("udp_port", 15783);
    this->declare_parameter<double>("timeout", 10.0);       // Timeout for alerting user of no packets
    this->declare_parameter<double>("max_age",  1.0);       // Max age of accepted packets

    std::string param_ip_address = this->get_parameter("ros_ip").as_string();
    uint16_t param_port = this->get_parameter("udp_port").as_int();
    
    t_Udp_timeout_ = this->get_parameter("timeout").as_double();
    t_Udp_maxAge_ = this->get_parameter("max_age").as_double();

    RCLCPP_INFO(this->get_logger(), "\tTransponder link at %s:%d",param_ip_address.c_str(), param_port);

    // Create a single UDP socket for both sending and receiving
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Transponder failed to create socket");
        rclcpp::shutdown();
    }

    // Set socket options (e.g., timeout for receiving)
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)) < 0) {
        std::cerr << "Failed to set socket options" << std::endl;
        close(sockfd_);
        exit(EXIT_FAILURE);
    }

    // Bind to the UDP socket for receiving data
    struct sockaddr_in recv_addr;
    std::memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = INADDR_ANY;
    recv_addr.sin_port = htons(param_port);

    if (bind(sockfd_, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to bind receive socket");
        rclcpp::shutdown();
    }

    // Set up the target address for sending data
    bzero(&send_addr_, sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_addr.s_addr = inet_addr(param_ip_address.c_str());
    send_addr_.sin_port = htons(param_port);  // Use the same port for sending

    // UDP socket listener thread
    receive_data_thread_ = std::thread(&transponder2ros::read_udpData, this);

    // All done
    return;
}

void transponder2ros::push_udp(StructIacTransponder data)
{
    // Convert data to a packet
    TransponderUdpPacket udp_packet;
    udp_packet.data = data;

    // Send data
    ssize_t sent_bytes = sendto(
        sockfd_, udp_packet.raw, SIZEOF_TransponderUdpPacket, 0,
         (sockaddr*)&send_addr_, sizeof(send_addr_)
    );

    if (sent_bytes < 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1UL * 1000 * 1000, "Failed to send telop UDP packet");
    }
    else
    {
        if (0)
        {
            RCLCPP_INFO(this->get_logger(), "Sent %ld bytes", sent_bytes);
            RCLCPP_INFO(this->get_logger(), "Time %f", udp_packet.data.utc);
        }
    }

    // Debugging
    if (0)
    {
        RCLCPP_INFO(this->get_logger(), "Sending %5.2f, %5.2f, %5.2f",
        udp_packet.data.lat, udp_packet.data.lon, udp_packet.data.vel);
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
        ssize_t len = recvfrom(sockfd_, buffer_, sizeof(buffer_) - 1, 0, (struct sockaddr *)&read_addr_, &addr_len_);

        if (len == SIZEOF_TransponderUdpPacket)
        {
            // RCLCPP_INFO(this->get_logger(), "Got %ld bytes", len);

            TransponderUdpPacket transponder;

            std::memcpy(&transponder.data, &buffer_, SIZEOF_TransponderUdpPacket);

            // Reject if the versions don't match
            if (transponder.data.version != TRANSPONDER_UDP_STRUCT_VERISON)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Version mismatch | car: %d, ours: 0x%02X, theirs: 0x%02X",
                    transponder.data.car_id, TRANSPONDER_UDP_STRUCT_VERISON, transponder.data.version
                );
            }

            // Reject if message is too old
            double msg_tDiff = this->get_clock()->now().seconds() - transponder.data.utc;

            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Latency: %.3f s", msg_tDiff);

            if (abs(msg_tDiff) > t_Udp_timeout_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                        "Transponder UPD packet late by %.3f s", msg_tDiff);
                continue;
            }

            // Publish the data
            transponder_msgs::msg::Transponder msg;

            msg.header.stamp.sec = static_cast<int32_t>(transponder.data.utc);
            msg.header.stamp.nanosec = static_cast<uint32_t>((transponder.data.utc - msg.header.stamp.sec) * 1e9);;

            msg.header.frame_id = "map";

            msg.car_id = transponder.data.car_id;
            msg.lat = transponder.data.lat;
            msg.lon = transponder.data.lon;
            msg.vel = transponder.data.vel;
            msg.state = transponder.data.state;

            pub_Transponder_->publish(msg);

            // Update our last received time
            t_last_packet_ = this->get_clock()->now();

            // Update user if reconnected
            if (has_timeout_)
            {
                RCLCPP_INFO(this->get_logger(), "Transponder connected");
                has_timeout_ = false;
            }

            // Debugging
            if (0)
            {
                RCLCPP_INFO(this->get_logger(), "lat: %8.3f, lon: %8.3f",
                    transponder.data.lat, transponder.data.lon
                );
            }

        }
    }

    // All done
    return;
}
