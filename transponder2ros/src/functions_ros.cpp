#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::callback_NavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    return;
}

void transponder2ros::callback_debug()
{
    // Push a fake packet out

    // Compile the UDP string
    udp_out_.data.utc = this->get_clock()->now().seconds();

    udp_out_.data.lat = udp_out_.data.lat+0.1;
    udp_out_.data.lon = udp_out_.data.lon-0.1;
    udp_out_.data.vel = udp_out_.data.vel + 0.2;
    udp_out_.data.state = VehicleStateTransponder::NOMINAL;

    push_udp();

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
    pubVersion_->publish(msg);

    // All done
    return;
}