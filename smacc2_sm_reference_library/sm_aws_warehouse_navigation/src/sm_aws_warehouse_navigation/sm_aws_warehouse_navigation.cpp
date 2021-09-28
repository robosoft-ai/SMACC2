#include "sm_aws_warehouse_navigation/sm_aws_warehouse_navigation.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<sm_aws_warehouse_navigation::SmAwsWarehouseNavigation>();
}
