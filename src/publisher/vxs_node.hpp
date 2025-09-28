/**
 * @file vx_sensor.hpp
 * @author George Terzakis (george.terzakis.ext@voxelsensors.com)
 * @brief VXS sesnor data publishing node
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef VXS_SENSOR_HPP
#define VXS_SENSOR_HPP

#include <memory>
#include <thread>
#include <string>

#include <thread>
#include <mutex>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <vxs_sensor_ros1/UpdateObservationWindow.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <SDK2.h>

using namespace std::chrono_literals;

namespace vxs_ros1
{
    struct CameraCalibration;

    //! Filtering parameters
    struct FilteringParams
    {
        static const int DEFAULT_BINNING = 0;
        static const float DEFAULT_PREFILTERING_THRESH; // = 2.0;
        static const float DEFAULT_FILTERP1;            // = 0.1;
        static const int DEFAULT_TEMPORAL_THRESH = 4;
        static const int DEFAULT_SPATIAL_THRESH = 10;

        int binning_amount = DEFAULT_BINNING;
        float prefiltering_threshold = DEFAULT_PREFILTERING_THRESH;
        float filterP1 = DEFAULT_FILTERP1;
        int temporal_threshold = DEFAULT_TEMPORAL_THRESH;
        int spatial_threshold = DEFAULT_SPATIAL_THRESH;
    };

    class VxsSensorPublisher
    {

    public:
        //! Sensor dimensions here. @TODO: Should be able to get that from the SDK?
        static const int SENSOR_WIDTH = 300;
        static const int SENSOR_HEIGHT = 300;

        VxsSensorPublisher(const ros::NodeHandle &nh, const ros::NodeHandle &nhp);
        ~VxsSensorPublisher();

    private:
        //! ROS public node handle
        ros::NodeHandle nh_;
        //! ROS private node handle
        ros::NodeHandle nhp_;

        //! Frame publishing thread
        std::shared_ptr<std::thread> frame_publishing_thread_;
        //! Frame polling thread
        std::shared_ptr<std::thread> frame_polling_thread_;

        std::shared_ptr<ros::Publisher> depth_publisher_;
        std::shared_ptr<ros::Publisher> cam_info_publisher_;
        std::shared_ptr<ros::Publisher> pcloud_publisher_;
        std::shared_ptr<ros::Publisher> evcloud_publisher_;

        //! FPS
        int fps_;
        //! Frame/streaming window in msec
        uint32_t period_;

        //! config json
        std::string config_json_;
        //! calibration json
        std::string calib_json_;

        //! Publish depth image
        bool publish_depth_image_;

        //! Publish pointcloud
        bool publish_pointcloud_;

        //! Publish events flag. This should override depth + simpple pointcloud publishers
        bool publish_events_;

        //! Shut down request flag
        bool flag_shutdown_request_;
        //! Flag indicating execution is inside the polling loop.
        bool flag_in_polling_loop_;
        //! A flag forcing update of the observation window wit the cached values
        std::atomic<bool> flag_update_observation_window_;
        //! observation window parameters
        int on_time_, period_time_;

        //! Camera #1 calibration
        std::vector<CameraCalibration> cams_;

        //! Filtering parameters
        FilteringParams filtering_params_;

        //! Observation window service server
        ros::ServiceServer observation_window_update_server_;

        //! Initializae sensor
        bool InitSensor();
        //! The main loop of the frame ppolling thread
        void FramePollingLoop();
        //! Unpack sensor data into a cv::Mat and return 3D points
        cv::Mat UnpackFrameSensorData(float *frameXYZ, std::vector<cv::Vec3f> &points);

        //! Load calilbration from json (required for the formation of the depth map)
        void LoadCalibrationFromJson(const std::string &calib_json);
        //! Publish image and calibration
        void PublishDepthImage(const cv::Mat &depth_image);
        //! Publish a pointcloud
        void PublishPointcloud(const std::vector<cv::Vec3f> &points);
        //! Pubish stamped pointcloud
        void PublishStampedPointcloud(const int N, vxsdk::vxXYZT *eventsXYZT);
        //! Update observation window callback
        bool UpdateObservationWindowCB(
            vxs_sensor_ros1::UpdateObservationWindow::Request &request,
            vxs_sensor_ros1::UpdateObservationWindow::Response &result);
    };

} // end namespace vxs_ros

#endif