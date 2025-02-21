#include "transponder2ros/transponder2ros.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<transponder2ros>());
  rclcpp::shutdown();
  return 0;
}
