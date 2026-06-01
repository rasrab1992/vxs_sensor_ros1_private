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
#include <shared_mutex>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <vxs_sensor_ros1/UpdateObservationWindow.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <SDK2.h>

using namespace std::chrono_literals;

namespace imu
{
    struct IMUSample;
}
namespace vxs_ros1
{
    enum class TSensorFrame
    {
        EventsXYZT = 0,
        FrameXYZ
    };

    struct RGBFrame
    {
        cv::Mat img;
        ros::Time stamp;
    };
    struct RawSensorFrame
    {
        //! number of bytes in the raw frame
        int N;
        //! Number of strruct/float entries
        int num_entries;
        //! Sensor frame type (events XYZT/frame XYZ)
        TSensorFrame frame_type;
        //! The frame data as a strteam of bytes
        std::shared_ptr<std::vector<uint8_t>> data;
        //! The global (ROS) stamp
        ros::Time stamp;
    };

    struct CameraCalibration;

    //! Filtering parameters
    struct FilteringParams
    {
        static const int DEFAULT_BINNING = 0;
        static const float DEFAULT_PREFILTERING_THRESH;    // = 2.0;
        static const int DEFAULT_POSTFILTERING_THRESH = 5; //

        static const float DEFAULT_FILTERP1X; // = 0.1;
        static const float DEFAULT_FILTERP1Y; // = 0.1;
        static const int DEFAULT_TEMPORAL_THRESH = 4;
        static const int DEFAULT_SPATIAL_THRESH = 10;

        static const int DEFAULT_MEDIAN_REJECTION_THRESH = 5; //

        int binning_amount = DEFAULT_BINNING;
        float prefiltering_threshold = DEFAULT_PREFILTERING_THRESH;
        int postfiltering_threshold = DEFAULT_POSTFILTERING_THRESH;

        float filterP1X = DEFAULT_FILTERP1X;
        float filterP1Y = DEFAULT_FILTERP1Y;
        int temporal_threshold = DEFAULT_TEMPORAL_THRESH;
        int spatial_threshold = DEFAULT_SPATIAL_THRESH;

        int median_rejection_threshold = DEFAULT_MEDIAN_REJECTION_THRESH;
    };

    class VxsSensorPublisher
    {

    public:
        //! Use this to convert long int to a double timestamp in seconds
        static constexpr double PERIOD_75_MHZ = 13.3333 * 1e-9;

        //! Sensor dimensions here. @TODO: Should be able to get that from the SDK?
        static const int SENSOR_WIDTH = 300;
        static const int SENSOR_HEIGHT = 300;

        //! Default RGB dimensions
        static const int DEFAULT_RGB_WIDTH = 640;
        static const int DEFAULT_RGB_HEIGHT = 480;

        //! Maximum depth of pubishing queue
        static const int MAX_QUEUE_DEPTH = 100;

        VxsSensorPublisher(const ros::NodeHandle &nh, const ros::NodeHandle &nhp);
        ~VxsSensorPublisher();

    private:
        //! ROS public node handle
        ros::NodeHandle nh_;
        //! ROS private node handle
        ros::NodeHandle nhp_;

        //! Frame publishing thread
        std::shared_ptr<std::thread> frame_publishing_thread_;
        //! Sensor frame polling thread
        std::shared_ptr<std::thread> frame_polling_thread_;
        //! RGB publishing thread
        std::shared_ptr<std::thread> rgb_publishing_thread_;

        std::shared_ptr<ros::Publisher> depth_publisher_;
        std::shared_ptr<ros::Publisher> cam_info_publisher_;
        std::shared_ptr<ros::Publisher> pcloud_publisher_;
        std::shared_ptr<ros::Publisher> evcloud_publisher_;
        std::shared_ptr<ros::Publisher> imu_publisher_;
        std::shared_ptr<ros::Publisher> gray_publisher_;

        //! FPS
        int fps_;
        //! Frame/streaming window in msec
        uint32_t period_;

        //! config json
        std::string config_json_;
        //! calibration json
        std::string calib_json_;
        //! RGB calibration (yaml)
        std::string rgb_calib_filename_;
        //! RGB pose wrt sensor calibration file (text)
        std::string rgb_pose_calib_filename_;
        //! RGB Intrinsics
        cv::Matx<float, 3, 3> rgbK_;
        //! RGB distortion
        cv::Vec<float, 5> rgbD_;
        //! RGB orientation with respect to sensr
        cv::Matx<float, 3, 3> rgbR_cs_;
        //! RGB translation with respect to sensor
        cv::Vec3f rgbt_cs_;
        //! RGB/Grayscale image size
        cv::Size_<int> rgb_image_size_;
        //! OpenCV capture object
        cv::VideoCapture cam_cap_;
        //! RGB camera index
        int rgb_cam_index_;

        //! Publish depth image
        bool publish_depth_image_;

        //! Publish images (actual grayscale images)
        bool publish_rgb_;
        //! Name of gray topic
        std::string gray_topic_ = "";

        //! Publish pointcloud
        bool publish_pointcloud_;

        //! Publish events flag. This should override depth + simpple pointcloud publishers
        bool publish_events_;

        //! Publish imu samples (if available)
        bool publish_imu_;
        //! Name of imu topic
        std::string imu_topic_ = "";

        //! Shut down request flag
        std::atomic<bool> flag_shutdown_request_;
        //! A flag forcing update of the observation window with the cached values
        std::atomic<bool> flag_update_observation_window_;
        //! observation window parameters
        int on_time_, period_time_;
        //! Mainloop sleep time
        int sleep_time_ms_;

        std::queue<RawSensorFrame> frame_queue_;
        std::mutex frame_queue_mutex_;
        std::condition_variable frame_queue_cv_;

        std::queue<RGBFrame> rgb_queue_;
        std::mutex rgb_queue_mutex_;
        std::condition_variable rgb_queue_cv_;

        //! frame counter
        int frame_counter_;

        //! Reference ros Time for both frames and imu samples.
        ros::Time ros_ref_time_;
        //! Current reference HW stamp in the sensor. Always is the latest depth (or IMU if depth was invalid)
        double sensor_ref_time_;
        //! The latest HW depth stamp
        double latest_depth_stamp_;
        //! Flag indicating that reference time is initialized
        bool flag_ref_time_initialized_;

        //! IMU wall-clock anchor (set before first frame to avoid USB camera delay offset)
        ros::Time    imu_ros_epoch_;
        double       imu_hw_epoch_         = 0.0;
        long long int imu_last_stamp_      = -1;
        bool         imu_anchor_set_       = false;

        //! Mutex for reference time members
        std::shared_timed_mutex ref_time_mutex_;

        //! Camera #1 calibration
        std::vector<CameraCalibration> cams_;

        //! Filtering parameters
        FilteringParams filtering_params_;

        //! Observation window service server
        ros::ServiceServer observation_window_update_server_;

        //! Initializae sensor
        bool InitSensor();
        //! Return pointer to next sensor frame
        void *GetNextSensorFrame(int &N);
        //! Return next captured RGB frame
        cv::Mat GetNextRGBFrame();

        //! The main loop of the frame polling thread
        void SensorPollingLoop();
        //! Publishing loop (for depth + pointclouds)
        void SensorPublishingLoop();
        //! Publishing loop for rgb images (as grayscale)
        void RGBPublishingLoop();

        //! Unpack sensor data into a cv::Mat and return 3D points
        cv::Mat UnpackFrameSensorData(float *frameXYZ, std::vector<cv::Vec3f> &points);

        //! Load calilbration from json (required for the formation of the depth map)
        void LoadCalibrationFromJson(const std::string &calib_json);

        //! Publish grayscale image and calibration
        void PublishGrayscaleImage(const cv::Mat &rgb, const ros::Time &stamp);
        //! Publish depth image and calibration
        void PublishDepthImage(const cv::Mat &depth_image, const ros::Time &depth_stamp);
        //! Publish a pointcloud
        void PublishPointcloud(const std::vector<cv::Vec3f> &points, const ros::Time &stamp);
        //! Pubish stamped pointcloud
        void PublishStampedPointcloud(const int N, vxsdk::vxXYZT *eventsXYZT, const ros::Time &stamp);
        //! Publish imu sample
        void PublishIMUSample(const imu::IMUSample &sample, const ros::Time &depth_stamp);
        //! Update observation window callback
        bool UpdateObservationWindowCB(
            vxs_sensor_ros1::UpdateObservationWindow::Request &request,
            vxs_sensor_ros1::UpdateObservationWindow::Response &result);
    };

} // end namespace vxs_ros

#endif