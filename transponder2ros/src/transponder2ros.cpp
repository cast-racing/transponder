#include "transponder2ros/transponder2ros.hpp"

transponder2ros::transponder2ros()
    : Node("transponder_node")
{

    RCLCPP_INFO(this->get_logger(), "Transponder Node");

    // Parameters
    this->declare_parameter("navsatfix_in", "/state/lla");
    this->declare_parameter("competitor_out", "/test");
    this->declare_parameter("version_out", "/transponder/version");
    this->declare_parameter("car_id", 1);

    std::string param_NavSatFix_in   = this->get_parameter("navsatfix_in").as_string();
    std::string param_competitor_out = this->get_parameter("competitor_out").as_string();
    std::string param_version_out = this->get_parameter("version_out").as_string();

    // Init variables
    udp_out_.data.version = TRANSPONDER_UDP_STRUCT_VERISON;
    udp_out_.data.car_id = this->get_parameter("car_id").as_int();

    // Init the UDP connection 
    init_udp();

    // Publishers
    pubCompetitorEsimate_ = this->create_publisher<iac_msgs::msg::CompetitorEstimate>(param_competitor_out, 1);
    pubVersion_ = this->create_publisher<std_msgs::msg::String>(param_version_out, 10);
    
    // Subscribers
    subNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        param_NavSatFix_in,
        1,
        std::bind(&transponder2ros::callback_NavSatFix, this, std::placeholders::_1)
    );

    // Timers
    timer_1Hz_ = this->create_wall_timer(
        std::chrono::seconds( 1),
        std::bind(&transponder2ros::callback_1Hz, this)
    );
 
    timer_10s_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&transponder2ros::callback_10s, this)
    );

    timer_debug_ = this->create_wall_timer(
        std::chrono::milliseconds(1*1000),
        std::bind(&transponder2ros::callback_debug, this)
    ); 

    // All done
    return;
}

transponder2ros::~transponder2ros()
{
    if (receive_data_thread_.joinable())
    {
        receive_data_thread_.join();
    }
    if (socket_fd != -1)
    {
        close(socket_fd);
    }
}