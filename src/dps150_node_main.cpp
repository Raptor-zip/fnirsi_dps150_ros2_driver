#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "fnirsi_dps150_driver/dps150_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fnirsi_dps150_driver::Dps150Node>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
