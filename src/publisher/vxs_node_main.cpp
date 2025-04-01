#include <vxs_node.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vxs_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    vxs_ros1::VxsSensorPublisher vxs_publisher(nh, nhp);

    // Now spin ...
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}