#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_udp()
{

    // Get the param
    this->declare_parameter("car_ip", "0.0.0.0");
    this->declare_parameter<int>("transponder.udp_port_read", 15782);
    this->declare_parameter<int>("transponder.udp_port_send", 15783);
    this->declare_parameter<double>("transponder.timeout", 1.0);

    std::string param_ip_address = this->get_parameter("car_ip").as_string();
    uint16_t param_portSend = this->get_parameter("transponder.udp_port_send").as_int();
    uint16_t param_portRead = this->get_parameter("transponder.udp_port_read").as_int();
    
    t_Udp_timeout_ = this->get_parameter("transponder.timeout").as_double();

    RCLCPP_INFO(this->get_logger(), "\tStreaming to transponder to %s:%d",param_ip_address.c_str(), param_portSend);

    // Creating of the UDP sockets could be cleaned up

    // Create a UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "transponder failed to create socket");
        rclcpp::shutdown();
    }

    // Set up the sender UDP
    bzero(&send_addr_,sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_addr.s_addr = inet_addr(param_ip_address.c_str());
    send_addr_.sin_port = htons(param_portSend);


    // Create the socket
    read_addr_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (read_addr_ < 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to create socket");
        rclcpp::shutdown();
    }

    // Set socket options
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (setsockopt(read_addr_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)) < 0)
    {
        std::cerr << "Failed to set socket options" << std::endl;
        close(read_addr_);
        exit(EXIT_FAILURE);
    }

    // Bind to the UDP socket
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(param_portRead);

    if (bind(read_addr_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to bind socket");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "\tListening to transponders on port:%d", param_portSend);

    // UDP socket listener
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
        ssize_t len = recvfrom(read_addr_, buffer_, sizeof(buffer_) - 1, 0, (struct sockaddr *)&client_addr_, &addr_len_);

        if (len == SIZEOF_TransponderUdpPacket)
        {
            // RCLCPP_INFO(this->get_logger(), "Got %ld bytes", len);

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

            // Publish the data
            transponder_msgs::msg::Transponder msg;

            rclcpp::Time utc(transponder.data.utc);
            msg.header.stamp.sec = utc.seconds();
            msg.header.stamp.nanosec = utc.nanoseconds();
        
            msg.header.frame_id = "map";

            msg.car_id = transponder.data.car_id;
            msg.lat = transponder.data.lat;
            msg.lon = transponder.data.lon;
            msg.vel = transponder.data.vel;
            msg.state = static_cast<uint8_t>(transponder.data.state);

            pub_Transponder_->publish(msg);

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

        }
    }

    // All done
    return;
}
