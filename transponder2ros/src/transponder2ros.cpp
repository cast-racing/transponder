#include "transponder2ros/transponder2ros.hpp"

transponder2ros::transponder2ros()
    : Node("transponder_node")
{

    RCLCPP_INFO(this->get_logger(), "Transponder Node");

    // Parameters
    this->declare_parameter("navsatfix_in", "/state/lla");
    this->declare_parameter("competitor_out", "/test");
    this->declare_parameter("car_id", 1);

    std::string param_NavSatFix_in   = this->get_parameter("navsatfix_in").as_string();
    std::string param_competitor_out = this->get_parameter("competitor_out").as_string();

    // Init variables
    udp_out_.data.version = TRANSPONDER_UDP_STRUCT_VERISON;
    udp_out_.data.car_id = this->get_parameter("car_id").as_int();

    // Init the UDP connection 
    init_udp();

    // Publishers
    pubCompetitorEsimate_ = this->create_publisher<iac_msgs::msg::CompetitorEstimate>(param_competitor_out, 1);
    
    // Subscribers
    subNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        param_NavSatFix_in,
        1,
        std::bind(&transponder2ros::callback_NavSatFix, this, std::placeholders::_1)
    );

    // Timers

    timer_debug_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&transponder2ros::callback_debug, this)
    ); 

    // All done
    return;
}
