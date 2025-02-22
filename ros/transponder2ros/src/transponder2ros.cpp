#include "transponder2ros/transponder2ros.hpp"

transponder2ros::transponder2ros()
    : Node("transponder_node")
{

    RCLCPP_INFO(this->get_logger(), "Transponder Node");

    // Parameters
    this->declare_parameter("car_id", 1);
    car_id_ = this->get_parameter("car_id").as_int();
    
    // Init the UDP connection 
    init_udp();

    // Init ROS
    init_ros();

    // All done
    return;
}

transponder2ros::~transponder2ros()
{
    if (receive_data_thread_.joinable())
    {
        receive_data_thread_.join();
    }
    if (read_addr_ != -1)
    {
        close(read_addr_);
    }
}