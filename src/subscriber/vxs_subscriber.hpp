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

#include <memory>
#include <thread>
#include <string>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

namespace vxs_ros1
{
    struct CameraCalibration;

    class VxsSensorSubscriber
    {

    public:
        //! Sensor dimensions here. @TODO: Should be able to get that from the SDK?
        static const int SENSOR_WIDTH = 300;
        static const int SENSOR_HEIGHT = 300;

        VxsSensorSubscriber(const ros::NodeHandle &nh, const ros::NodeHandle &nhp);
        ~VxsSensorSubscriber();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nhp_;

        std::shared_ptr<ros::Subscriber> depth_subscriber_;
        std::shared_ptr<ros::Subscriber> cam_info_subscriber_;
        std::shared_ptr<ros::Subscriber> pcloud_subscriber_;
        std::shared_ptr<ros::Subscriber> evcloud_subscriber_;

        //! Camera #1 calibration
        std::shared_ptr<CameraCalibration> cam_;

        void CameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg);
        void DepthImageCB(const sensor_msgs::Image::ConstPtr &depth_img_msg);
        void PointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &depth_img_msg);
        void StampedPointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg);
    };

} // end namespace vxs_ros

#endif