/**
 * @file vx_sensor.hpp
 * @author George Terzakis (george.terzakis.ext@voxelsensors.com)
 * @brief Example C++ subscriber to vxs_sensor data (pointcloud + depth image + camera info)
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef VXS_SUBSCRIBER_HPP
#define VXS_SUBSCRIBER_HPP

#include <condition_variable>
#include <memory>
#include <thread>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std::chrono_literals;

namespace vxs_ros
{
    struct CameraCalibration;

    class VxsSensorSubscriber : public rclcpp::Node
    {

    public:
        //! Sensor dimensions here. @TODO: Should be able to get that from the SDK?
        static const int SENSOR_WIDTH = 300;
        static const int SENSOR_HEIGHT = 300;

        VxsSensorSubscriber();
        ~VxsSensorSubscriber();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr evcloud_subscriber_;

        //! Camera #1 calibration
        std::shared_ptr<CameraCalibration> cam_;

        void CameraInfoCB(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);
        void DepthImageCB(const sensor_msgs::msg::Image::SharedPtr depth_img_msg);
        void PointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr depth_img_msg);
        void StampedPointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg);
    };

} // end namespace vxs_ros

#endif