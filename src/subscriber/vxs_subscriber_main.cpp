#include "subscriber/vxs_subscriber.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vxs_ros::VxsSensorSubscriber>());
    rclcpp::shutdown();
    return 0;
}