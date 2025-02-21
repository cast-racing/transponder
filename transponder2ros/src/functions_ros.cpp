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