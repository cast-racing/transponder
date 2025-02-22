#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_ros()
{
    // Parameters
    this->declare_parameter("transponder_in", "/transponder/in");
    this->declare_parameter("transponder_out", "/transponder/out");
    this->declare_parameter("version_out", "/transponder/version");

    std::string param_transponderIn = this->get_parameter("transponder_in").as_string();
    std::string param_transponderOut = this->get_parameter("transponder_out").as_string();
    std::string param_versionOut = this->get_parameter("version_out").as_string();

    // Publishers
    pub_Transponder_ = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderOut, 1);
    pub_Version_ = this->create_publisher<std_msgs::msg::String>(param_versionOut, 10);

    // Subscribers
    sub_Transponder_ = this->create_subscription<transponder_msgs::msg::Transponder>(
        param_transponderIn,
        1,
        std::bind(&transponder2ros::callback_Transponder, this, std::placeholders::_1));

    // Timers
    timer_1Hz_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&transponder2ros::callback_1Hz, this));

    timer_10s_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&transponder2ros::callback_10s, this));

    timer_debug_ = this->create_wall_timer(
        std::chrono::milliseconds(1 * 1000),
        std::bind(&transponder2ros::callback_debug, this));
}

void transponder2ros::callback_Transponder(const transponder_msgs::msg::Transponder::SharedPtr msg)
{
    // Need to send Transponder data from here
    StructIacTransponder transponder;

    rclcpp::Time utc(msg->header.stamp);

    transponder.version = TRANSPONDER_UDP_STRUCT_VERISON;                   // Struct version
    transponder.utc = utc.seconds();                                        // UTC time [ s ]
    transponder.car_id = car_id_;                                           // Car ID [ - ]
    transponder.lat = msg->lat;                                             // Vehicle longitude [ dd.dd ]
    transponder.lon = msg->lon;                                             // Vehicle latitude [ dd.dd ]
    transponder.vel = msg->vel;                                             // Vehicle speed [ m/s ]
    transponder.state = static_cast<VehicleStateTransponder>(msg->state);   // Vehicle state [ - ]
  
    // Push data
    push_udp(transponder);

    // Done
    return;
}

void transponder2ros::callback_debug()
{
    // Push a fake packet out

    static StructIacTransponder transponder;

    transponder.version = TRANSPONDER_UDP_STRUCT_VERISON;    // Struct version
    transponder.utc = this->get_clock()->now().seconds();    // UTC time [ s ]
    transponder.car_id = car_id_;                            // Car ID [ - ]
    transponder.lat = transponder.lat + 0.1;                 // Vehicle longitude [ dd.dd ]
    transponder.lon = transponder.lon - 0.1;                 // Vehicle latitude [ dd.dd ]
    transponder.vel = transponder.vel + 0.2;                 // Vehicle speed [ m/s ]
    transponder.state = VehicleStateTransponder::NOMINAL;    // Vehicle state [ - ]
    
    // Push data
    push_udp(transponder);

    // Done
    return;
}

void transponder2ros::callback_1Hz()
{

    rclcpp::Duration t_since_last_msg = this->get_clock()->now() - t_last_flag_;
    static bool notified_timeout_silence = false;

    // Check timeouts (only used for printing in this case)
    // RCLCPP_INFO(this->get_logger(), "t_since_last_msg: %6.3f",t_since_last_msg.seconds());

    if (t_since_last_msg.seconds() > t_Udp_timeout_)
    {
        has_timeout_ = true;
    }
    else
    {
        notified_timeout_silence = false;
    }

    // Print message for up to 30 s
    if (has_timeout_ && t_since_last_msg.seconds() < 30.0)
    {
        // Timeout
        RCLCPP_WARN(this->get_logger(), "No UDP transponder data for %.1f s", t_since_last_msg.seconds());
    }
    else if (t_since_last_msg.seconds() > 30.0 && notified_timeout_silence == 0)
    {
        RCLCPP_INFO(this->get_logger(), ">> Silencing transponder timeout message");
        notified_timeout_silence = true;
    }

    // All done
    return;
}

void transponder2ros::callback_10s()
{

    // RCLCPP_WARN(this->get_logger(), "10 s callback");

    // Publish a version string for debugging
    std::string software_version = GIT_INFO;
    std_msgs::msg::String msg;
    msg.data = "transponder (UDP): " + software_version;
    pub_Version_->publish(msg);

    // All done
    return;
}
