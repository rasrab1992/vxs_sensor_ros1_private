#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <ostream>

#include "publisher/vxs_node.hpp"
#include "common.hpp"
#include "imu.hpp"


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace
{
    //! Load RGB calibration from **yaml**
    bool LoadRGBCalibration(               //
        const std::string &rgb_calib_yaml, //
        const cv::Size_<int> &image_size,  //
        cv::Matx<float, 3, 3> &K,          //
        cv::Vec<float, 5> &D)
    {
        cv::FileStorage fs(rgb_calib_yaml, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            return false;
        }

        int rgb_calib_width, rgb_calib_height;
        fs["width"] >> rgb_calib_width;
        fs["height"] >> rgb_calib_height;

        float scaler_x = 1;
        float scaler_y = 1;
        if (image_size.width != rgb_calib_width || image_size.height != rgb_calib_height)
        {
            scaler_x = (1.0 * image_size.width) / rgb_calib_width;
            scaler_y = (1.0 * image_size.height / rgb_calib_height);
        }

        cv::Matx<float, 1, 5> mxD;
        fs["D"] >> mxD;
        for (int i = 0; i < 5; i++)
        {
            D[i] = mxD(0, i);
        }

        fs["K"] >> K;
        K(0, 0) *= scaler_x;
        K(0, 2) *= scaler_x;
        K(1, 1) *= scaler_y;
        K(1, 2) *= scaler_y;

        // fs["P"] >> P;
        /// P(0, 0) *= scaler_x;
        // P(0, 2) *= scaler_x;
        // P(1, 1) *= scaler_y;
        /// P(1, 2) *= scaler_y;

        // fs["R"] >> rgbR;

        fs.release();
        return true;
    }

    std::vector<std::string> SplitLine(const std::string &line)
    {
        // remove spaces from line
        std::string new_line;
        for (size_t i = 0; i < line.length(); i++)
        {
            if (line[i] != ' ')
            {
                new_line += line[i];
            }
        }
        std::istringstream iss(new_line);
        std::string substring;
        std::vector<std::string> tokens;
        while (std::getline(iss, substring, ','))
        {
            tokens.push_back(substring);
        }
        return tokens;
    }

    bool LoadRGBRelativePose(const std::string &pose_calib_filename, cv::Matx<float, 3, 3> &R_cs, cv::Vec3f &t_cs)
    {
        std::ifstream input(pose_calib_filename);
        if (!input.is_open())
        {
            return false;
        }
        std::string line;
        /////////// A. read 1st line with comma delimeted axis-angle params
        //
        std::getline(input, line);
        // Extract line #1 tokens
        std::vector<std::string> line1_strings = SplitLine(line);
        cv::Vec3f u = {static_cast<float>(atof(line1_strings[0].c_str())), static_cast<float>(atof(line1_strings[1].c_str())), static_cast<float>(atof(line1_strings[2].c_str()))};
        cv::Rodrigues(u, R_cs);
        //
        /////////// B. Read second line with translation (in mms)
        //
        //
        std::getline(input, line);
        // Extract line #2 tokens
        std::vector<std::string> line2_strings = SplitLine(line);
        t_cs = {static_cast<float>(atof(line2_strings[0].c_str())), static_cast<float>(atof(line2_strings[1].c_str())), static_cast<float>(atof(line2_strings[2].c_str()))};
        // t_cs *= 0.001;
        //
        //

        // Here, we could read the rest of the lines with RGB calibration but not needed.
        // std::getline(input, line);
        input.close();

        return true;
    }
}

namespace vxs_ros1
{

    const float FilteringParams::DEFAULT_PREFILTERING_THRESH = 2.0;
    const float FilteringParams::DEFAULT_FILTERP1X = 0.1;
    const float FilteringParams::DEFAULT_FILTERP1Y = 0.1;

    VxsSensorPublisher::VxsSensorPublisher(const ros::NodeHandle &nh,
                                           const ros::NodeHandle &nhp) : nh_(nh),                                                //
                                                                         nhp_(nhp),                                              //
                                                                         frame_polling_thread_(nullptr),                         //
                                                                         flag_shutdown_request_(false),                          //
                                                                         flag_update_observation_window_(false),                 //
                                                                         flag_ref_time_initialized_(false),                      //
                                                                         observation_window_update_server_(                      //
                                                                             nhp_.advertiseService(                              //
                                                                                 "update_observation_window",                    //
                                                                                 &VxsSensorPublisher::UpdateObservationWindowCB, //
                                                                                 this))
    {
        std::string package_directory = ros::package::getPath("vxs_sensor_ros1");
        ROS_INFO_STREAM("Package directory: " << package_directory);
        // Declare & Get parameters
        nhp.param<bool>("publish_depth_image", publish_depth_image_, false);
        nhp.param<bool>("publish_pointcloud", publish_pointcloud_, false);
        nhp.param<bool>("publish_events", publish_events_, false);
        nhp.param<bool>("publish_imu", publish_imu_, false);
        nhp.param<int>("fps", fps_, 20);
        period_ = std::lround(1000.0f / fps_); // period in ms (will be used in initialization if streaming events)

        nhp.param<int>("sleep_time_ms", sleep_time_ms_, 1);
        ROS_INFO_STREAM("Sleep time in waiting loop: " << sleep_time_ms_ << " ms.");

        // Observation window — must be initialized before InitSensor runs
        nhp.param<int>("on_time", on_time_, 0);
        nhp.param<int>("period_time", period_time_, 0);
        ROS_INFO_STREAM("Observation window: on_time=" << on_time_ << " period_time=" << period_time_);

        if (publish_events_)
        {
            // Disable both standard pointcloud and depth image publishing
            publish_depth_image_ = publish_pointcloud_ = false;
            ROS_INFO_STREAM("Streaming mode (event based) enabled. Disabling depth and standard pointcloud publishers.\n");
        }
        else
        {
            // Force pointcloud publishing by default if running frame based mode
            if (!publish_depth_image_ && !publish_pointcloud_)
            {
                publish_depth_image_ = true;
                ROS_INFO_STREAM("Running frame based mode. Enabling pointcloud publisher...\n");
            }
            ROS_INFO_STREAM("Pointcloud publisher: " << (publish_pointcloud_ ? "ENABLED." : "DISABLED.") << std::endl);
            ROS_INFO_STREAM("Depth image publisher: " << (publish_depth_image_ ? "ENABLED." : "DISABLED.") << std::endl);
            /*
            if (publish_imu_)
            {
                publish_imu_ = false;
                ROS_INFO_STREAM("IMU sample will **NOT** be published in frame mode... ");
            }
                */
        }

        ROS_INFO_STREAM("SDK Mode: " << (publish_events_ ? "STREAM." : "FRAME.") << std::endl);
        ROS_INFO_STREAM("Publish IMU: " << (publish_imu_ ? "YES." : "NO.") << std::endl);

        nhp.param<std::string>("config_json", config_json_, "config/and2_median_golden.json");
        nhp.param<std::string>("calib_json", calib_json_, "config/default_calib.json");

        // Filtering parameters
        nhp.param<int>("binning_amount", filtering_params_.binning_amount, FilteringParams::DEFAULT_BINNING);
        nhp.param<float>("prefiltering_threshold", filtering_params_.prefiltering_threshold, FilteringParams::DEFAULT_PREFILTERING_THRESH);
        nhp.param<int>("postfiltering_threshold", filtering_params_.postfiltering_threshold, FilteringParams::DEFAULT_POSTFILTERING_THRESH);

        nhp.param<float>("filterP1X", filtering_params_.filterP1X, FilteringParams::DEFAULT_FILTERP1X);
        nhp.param<float>("filterP1Y", filtering_params_.filterP1Y, FilteringParams::DEFAULT_FILTERP1Y);

        nhp.param<int>("temporal_threshold", filtering_params_.temporal_threshold, FilteringParams::DEFAULT_TEMPORAL_THRESH);
        nhp.param<int>("spatial_threshold", filtering_params_.spatial_threshold, FilteringParams::DEFAULT_SPATIAL_THRESH);

        nhp.param<int>("median_rejection_threshold", filtering_params_.median_rejection_threshold, FilteringParams::DEFAULT_MEDIAN_REJECTION_THRESH);

        nhp.param<bool>("publish_rgb", publish_rgb_, false);
        nhp.param<int>("rgb_width", rgb_image_size_.width, DEFAULT_RGB_WIDTH);
        nhp.param<int>("rgb_height", rgb_image_size_.height, DEFAULT_RGB_HEIGHT);
        nhp.param<int>("rgb_cam_index", rgb_cam_index_, -1);

        nhp.param<std::string>("rgb_calib", rgb_calib_filename_, "");
        nhp.param<std::string>("rgb_pose_calib", rgb_pose_calib_filename_, "");

        // names of imu and rgb topics (non-private!)
        nhp.param<std::string>("gray_topic", gray_topic_, "");
        nhp.param<std::string>("imu_topic", imu_topic_, "");

        // Print param values
        ROS_INFO_STREAM("Publish frame-based depth image (publlish_depth_image): " << (publish_depth_image_ ? "YES." : "NO."));
        ROS_INFO_STREAM("Publish frame-based pointcloud (publish_pcloud): " << (publish_pointcloud_ ? "YES." : "NO."));
        ROS_INFO_STREAM("Publish streaming based (stamped) point cloud (publish_events): " << (publish_events_ ? "YES." : "NO."));
        ROS_INFO_STREAM("Publish grayscale images: " << (publish_rgb_ ? "YES." : "NO."));
        if (publish_rgb_)
        {
            ROS_INFO_STREAM("----RGB camera index: " << rgb_cam_index_);
            ROS_INFO_STREAM("----RGB calibration yaml: " << rgb_calib_filename_);
            ROS_INFO_STREAM("----RGB Relative pose calibration yaml: " << rgb_pose_calib_filename_);
            ROS_INFO_STREAM("----RGB image size: (" << rgb_image_size_.width << ", " << rgb_image_size_.height << ")");
        }
        ROS_INFO_STREAM("FPS: " << fps_ << " and period in ms: " << period_);
        ROS_INFO_STREAM("Config JSON: " << config_json_);
        ROS_INFO_STREAM("Sensor calibration JSON: " << calib_json_);

        ROS_INFO_STREAM("Non-default grayscale topic name: " << gray_topic_);
        ROS_INFO_STREAM("Non-default imu topic: " << imu_topic_);

        // Load calibration into members
        LoadCalibrationFromJson(calib_json_);

        // Initialize Sensor
        if (!InitSensor())
        {
            ROS_ERROR_STREAM("Sensor initialization failed!");
            ros::shutdown();
        }

        // Create publishers
        depth_publisher_ = nullptr;
        if (publish_depth_image_)
        {
            depth_publisher_ = std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::Image>("depth/image", 10));
        }

        pcloud_publisher_ = nullptr;
        if (publish_pointcloud_)
        {
            pcloud_publisher_ = std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::PointCloud2>("pcloud/cloud", 10));
        }

        gray_publisher_ = nullptr;
        if (publish_rgb_)
        {
            if (gray_topic_.length() > 0)
            {
                gray_publisher_ = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::Image>(gray_topic_, 10));
            }
            else
            {
                gray_publisher_ = std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::Image>("mono/image", 10));
            }
        }

        evcloud_publisher_ = publish_events_ ? std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::PointCloud2>("pcloud/events", 10)) : nullptr;

        cam_info_publisher_ = std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::CameraInfo>("sensor/camera_info", 10));

        imu_publisher_ = nullptr;
        if (publish_imu_)
        {
            if (imu_topic_.length() > 0)
            {
                imu_publisher_ = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10));
            }
            else
            {
                imu_publisher_ = std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::Imu>("imu", 10));
            }
        }

        //! Start the pointcloud publishing thread
        ROS_INFO_STREAM("Starting frame publishing thread...");
        frame_publishing_thread_ = std::make_shared<std::thread>(std::bind(&VxsSensorPublisher::SensorPublishingLoop, this));
        ROS_INFO_STREAM("Done.");
        //! Start the rgb publishing thread
        rgb_publishing_thread_ = nullptr;
        if (publish_rgb_)
        {
            ROS_INFO_STREAM("Starting frame publishing thread...");
            rgb_publishing_thread_ = std::make_shared<std::thread>(std::bind(&VxsSensorPublisher::RGBPublishingLoop, this));
            ROS_INFO_STREAM("Done.");
        }
        // Initialize & start polling thread
        ROS_INFO_STREAM("Starting polling thread...");
        frame_polling_thread_ = std::make_shared<std::thread>(std::bind(&VxsSensorPublisher::SensorPollingLoop, this));
        ROS_INFO_STREAM("Done.");
    }

    VxsSensorPublisher::~VxsSensorPublisher()
    {
        flag_shutdown_request_ = true;
        frame_queue_cv_.notify_one();

        if (frame_polling_thread_)
        {
            if (frame_polling_thread_->joinable())
            {
                frame_polling_thread_->join();
            }
        }
        frame_polling_thread_ = nullptr;

        if (frame_publishing_thread_)
        {
            if (frame_publishing_thread_->joinable())
            {
                frame_publishing_thread_->join();
            }
        }
        frame_publishing_thread_ = nullptr;

        if (rgb_publishing_thread_)
        {
            rgb_queue_cv_.notify_one();
            if (rgb_publishing_thread_->joinable())
            {
                rgb_publishing_thread_->join();
            }
        }
        rgb_publishing_thread_ = nullptr;

        vxsdk::vxStopSystem();
    }

    bool VxsSensorPublisher::InitSensor()
    {
        // Detect sensor
        vxsdk::vxCameraType cam_type = vxsdk::vxDetectCameras();
        CV_Assert(cam_type != vxsdk::vxCameraType::none && "Failed detecting a camera type.");

        // Set the frame rate (or time window)
        vxsdk::vxFlag init_flags;
        if (publish_events_)
        {
            init_flags = vxsdk::vxFlag::XYZT;
            vxsdk::vxSetStreamingDuration(period_);
        }
        else
        {
            init_flags = vxsdk::vxFlag::FBPOINTCLOUD;
            vxsdk::vxSetFPS(fps_);
        }
        if (publish_imu_)
        {
            init_flags = init_flags | vxsdk::vxFlag::IMU;
        }
        // Set filtering parameters
        vxsdk::vxSetBinningAmount(filtering_params_.binning_amount);
        vxsdk::vxSetFilteringParameters(                  //
            filtering_params_.prefiltering_threshold,     //
            filtering_params_.postfiltering_threshold,    //
            filtering_params_.median_rejection_threshold, //
            filtering_params_.filterP1X,                  //
            filtering_params_.filterP1Y,                  //
            filtering_params_.temporal_threshold,         //
            filtering_params_.spatial_threshold);

        // Check if we need to start the webcamera
        if (publish_rgb_)
        {
            // Load RGB calibration
            std::cout << "Loading rgb calibration...\n";
            LoadRGBCalibration(rgb_calib_filename_, rgb_image_size_, rgbK_, rgbD_);
            std::cout << "Done.\n";
            // Load sensor to camera relative pose
            std::cout << "Loading rgb relative pose calibration...\n";
            LoadRGBRelativePose(rgb_pose_calib_filename_, rgbR_cs_, rgbt_cs_);
            std::cout << "Done.\n";
            // Start the capture
            cam_cap_.open(rgb_cam_index_, cv::CAP_ANY);
            if (!cam_cap_.isOpened())
            {
                std::cerr << "Unable to open camera with " << rgb_cam_index_ << ". Exiting ..." << std::endl;
                vxsdk::vxStopSystem();
                exit(0);
            }
            cam_cap_.set(cv::CAP_PROP_FRAME_WIDTH, rgb_image_size_.width);
            cam_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, rgb_image_size_.height);
        }

        // Start the SDK Engine.
        int cam_num = vxsdk::vxStartSystem( //
            config_json_.c_str(),           //
            calib_json_.c_str(),            //
            init_flags,
            cam_type);

        // Set observation window AFTER vxStartSystem (new SDK requires this order)
        if (on_time_ > 0 && period_time_ > 0)
        {
            vxsdk::vxSetObservationWindow(on_time_, period_time_);
            ROS_INFO("Observation window set after vxStartSystem: on_time=%d period_time=%d",
                     on_time_, period_time_);
        }

        return cam_num > 0;
    }

    void *VxsSensorPublisher::GetNextSensorFrame(int &N)
    {
        void *frame_ptr = nullptr;
        latest_depth_stamp_ = 0; // reset in order to check later
        while (!vxsdk::vxCheckForData())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        if (publish_events_) // streaming based publishing
        {
            vxsdk::vxXYZT *eventsXYZT = vxsdk::vxGetXYZT(N);
            frame_ptr = N > 0 ? (void *)eventsXYZT : nullptr;
            if (frame_ptr)
                latest_depth_stamp_ = eventsXYZT[0].timestamp * PERIOD_75_MHZ;
        }
        else // Frame based data
        {
            // Get data from the sensor
            long long int stamp;
            float *frameXYZ = vxsdk::vxGetFrameXYZ(stamp);
            if (!frameXYZ)
                std::cerr << "NULL XYZ!\n";

            frame_ptr = stamp == 0 ? nullptr : (void *)frameXYZ;
            if (frame_ptr)
                latest_depth_stamp_ = stamp * PERIOD_75_MHZ;
        }
        return frame_ptr;
    }

    cv::Mat VxsSensorPublisher::GetNextRGBFrame()
    {
        cv::Mat frame;

        cam_cap_.read(frame);

        // Create the camera frame and add to queue
        cv::Mat img = frame.clone();
        return img;
    }

    void VxsSensorPublisher::SensorPollingLoop()
    {
        // Anchor IMU hardware clock to wall time BEFORE blocking on the first
        // VXS frame. The USB camera starts timestamping immediately at launch;
        // if we wait until the first frame arrives (~150 ms later) the anchor
        // is late by that amount, causing a large t_cam_imu offset in Kalibr.
        if (publish_imu_)
        {
            int n = 0;
            vxsdk::vxIMU *ptr = vxsdk::vxGetIMU(n);
            if (n > 0 && ptr != nullptr)
            {
                double batch_span = (ptr[n-1].timestamp - ptr[0].timestamp) * PERIOD_75_MHZ;
                imu_ros_epoch_  = ros::Time::now() - ros::Duration(batch_span);
                imu_hw_epoch_   = ptr[0].timestamp * PERIOD_75_MHZ;
                imu_last_stamp_ = ptr[n-1].timestamp;
                imu_anchor_set_ = true;
                ROS_INFO("IMU anchor set (pre-loop): %d samples, span=%.3fs", n, batch_span);
            }
        }

        int counter = 0;
        while (!flag_shutdown_request_)
        {
            // Block here until the SDK has data ready, then fetch immediately.
            // Do NOT use std::async — concurrent SDK calls corrupt USB buffers.
            int N = -1;
            RawSensorFrame frame_data;
            // Capture wall time BEFORE blocking in GetNextSensorFrame so the
            // stamp is as close as possible to the actual hardware capture moment.
            // This reduces the fraction of IMU timestamps computed as "future".
            frame_data.stamp = ros::Time::now();
            void *frame_ptr = GetNextSensorFrame(N);
            // Update stamp to the time GetNextSensorFrame returned (actual arrival)
            frame_data.stamp = ros::Time::now();
            cv::Mat rgb_img;
            if (publish_rgb_)
            {
                rgb_img = GetNextRGBFrame();
            }

            if (publish_events_) // streaming based publishing
            {
                frame_data.N = N * sizeof(vxsdk::vxXYZT);
                frame_data.num_entries = N;
                frame_data.frame_type = TSensorFrame::EventsXYZT;
            }
            else // Frame based data
            {
                counter++;
                frame_data.N = SENSOR_WIDTH * SENSOR_HEIGHT * sizeof(float);
                frame_data.num_entries = SENSOR_WIDTH * SENSOR_HEIGHT;
                frame_data.frame_type = TSensorFrame::FrameXYZ;
            }

            if (frame_ptr)
            {
                // Copy into the frame queue and release the publishing thread
                frame_data.data = std::make_shared<std::vector<uint8_t>>();
                frame_data.data->resize(frame_data.N);

                std::memcpy((void *)&(*frame_data.data)[0], frame_ptr, frame_data.N);

                std::unique_lock<std::mutex> lock(frame_queue_mutex_);
                if (frame_queue_.size() >= MAX_QUEUE_DEPTH)
                {
                    frame_queue_.pop();
                }
                frame_queue_.push(frame_data);
                frame_queue_cv_.notify_one();
            }

            // Collect and publish IMU samples directly (no worker thread).
            if (publish_imu_)
            {
                int num_samples = 0;
                vxsdk::vxIMU *sample_ptr = vxsdk::vxGetIMU(num_samples);

                if (num_samples > 0 && sample_ptr != nullptr)
                {
                    if (!imu_anchor_set_)
                    {
                        // Fallback anchor if pre-loop anchor didn't fire.
                        double batch_span = (sample_ptr[num_samples-1].timestamp - sample_ptr[0].timestamp) * PERIOD_75_MHZ;
                        imu_ros_epoch_  = ros::Time::now() - ros::Duration(batch_span);
                        imu_hw_epoch_   = sample_ptr[0].timestamp * PERIOD_75_MHZ;
                        imu_last_stamp_ = sample_ptr[num_samples - 1].timestamp;
                        imu_anchor_set_ = true;
                        ROS_INFO("IMU anchor set (fallback): %d samples, span=%.3fs", num_samples, batch_span);
                    }
                    else
                    {
                        int new_count = 0;
                        for (int i = 0; i < num_samples; i++)
                        {
                            if (sample_ptr[i].timestamp > imu_last_stamp_)
                            {
                                imu::IMUSample s(sample_ptr[i]);
                                ros::Time stamp = imu_ros_epoch_ + ros::Duration(s.stamp_seconds - imu_hw_epoch_);
                                PublishIMUSample(s, stamp);
                                new_count++;
                            }
                        }
                        if (new_count > 0)
                            imu_last_stamp_ = sample_ptr[num_samples - 1].timestamp;

                        ROS_INFO_THROTTLE(5.0, "IMU: buf=%d  new=%d", num_samples, new_count);
                    }
                }
            }

            if (publish_rgb_ && flag_ref_time_initialized_)
            {
                RGBFrame rgb_data;
                rgb_data.stamp = frame_data.stamp; // = ros_ref_time_
                rgb_data.img = rgb_img;
                std::unique_lock<std::mutex> lock(rgb_queue_mutex_);
                if (rgb_queue_.size() >= MAX_QUEUE_DEPTH)
                {
                    rgb_queue_.pop();
                }
                rgb_queue_.push(rgb_data);
                rgb_queue_cv_.notify_one();
            }

            // Observation window changes are applied at next sensor restart (not mid-stream)
            flag_update_observation_window_ = false;
        }
    }

    cv::Mat VxsSensorPublisher::UnpackFrameSensorData(float *frameXYZ, std::vector<cv::Vec3f> &points)
    {
        // Use cam #1 intrinsics for the depth image sensor
        const float &fx = cams_[0].K(0, 0);
        const float &fy = cams_[0].K(1, 1);
        const float &cx = cams_[0].K(0, 2);
        const float &cy = cams_[0].K(1, 2);
        cv::Mat depth(SENSOR_HEIGHT, SENSOR_WIDTH, CV_16U);
        depth = 0;
        points.clear();
        for (size_t r = 0; r < SENSOR_HEIGHT; r++)
        {
            for (size_t c = 0; c < SENSOR_WIDTH; c++)
            {
                const float &Z = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 2];
                if (Z > 1e-5)
                {
                    const float &X = frameXYZ[(r * SENSOR_WIDTH + c) * 3];
                    const float &Y = frameXYZ[(r * SENSOR_WIDTH + c) * 3 + 1];

                    // Keep the point, irrespective of visibility on sensor (it shouldn't be happening though...)
                    points.emplace_back(X, Y, Z);

                    const int x = std::lround(X / Z * fx + cx);
                    const int y = std::lround(Y / Z * fy + cy);

                    //  Check for negatives and out-of-bounds
                    if (y < 0 || y > SENSOR_HEIGHT - 1 || //
                        x < 0 || x > SENSOR_WIDTH - 1)
                    {
                        continue;
                    }

                    // Get a 16-bit approximation and save at x, y location
                    uint16_t iZ = std::lround(Z);
                    depth.at<uint16_t>(y, x) = iZ;
                }
            }
        }
        return depth;
    }

    void VxsSensorPublisher::LoadCalibrationFromJson(const std::string &calib_json)
    {
        // @TODO: Read the config to acquire number of cameras! Assuming stereo for now....
        cams_.resize(2);
        cv::FileStorage fs(calib_json, 0);
        cv::FileNode root = fs["Cameras"];
        cv::FileNode cam1 = root[0];
        cams_[0].t = cv::Vec3f({cam1["Translation"][0], cam1["Translation"][1], cam1["Translation"][2]});           //
        cams_[0].R = cv::Matx<float, 3, 3>({cam1["Rotation"][0][0], cam1["Rotation"][0][1], cam1["Rotation"][0][2], //
                                            cam1["Rotation"][1][0], cam1["Rotation"][1][1], cam1["Rotation"][1][2], //
                                            cam1["Rotation"][2][0], cam1["Rotation"][2][1], cam1["Rotation"][2][2]});
        cams_[0].dist = cv::Vec<float, 5>({cam1["Distortion"][0], cam1["Distortion"][1], cam1["Distortion"][2], cam1["Distortion"][3], cam1["Distortion"][4]});
        cams_[0].K = cv::Matx<float, 3, 3>({cam1["Intrinsic"][0][0], cam1["Intrinsic"][0][1], cam1["Intrinsic"][0][2], //
                                            cam1["Intrinsic"][1][0], cam1["Intrinsic"][1][1], cam1["Intrinsic"][1][2], //
                                            cam1["Intrinsic"][2][0], cam1["Intrinsic"][2][1], cam1["Intrinsic"][2][2]});
        cams_[0].image_size = cv::Size_<int>(cam1["SensorSize"]["Width"], cam1["SensorSize"]["Height"]);

        cv::FileNode cam2 = root[1];
        cams_[1].t = cv::Vec3f({cam2["Translation"][0], cam2["Translation"][1], cam2["Translation"][2]});           //
        cams_[1].R = cv::Matx<float, 3, 3>({cam2["Rotation"][0][0], cam2["Rotation"][0][1], cam2["Rotation"][0][2], //
                                            cam2["Rotation"][1][0], cam2["Rotation"][1][1], cam2["Rotation"][1][2], //
                                            cam2["Rotation"][2][0], cam2["Rotation"][2][1], cam2["Rotation"][2][2]});
        cams_[1].dist = cv::Vec<float, 5>({cam2["Distortion"][0], cam2["Distortion"][1], cam2["Distortion"][2], cam2["Distortion"][3], cam1["Distortion"][4]});
        cams_[1].K = cv::Matx<float, 3, 3>({cam2["Intrinsic"][0][0], cam2["Intrinsic"][0][1], cam2["Intrinsic"][0][2], //
                                            cam2["Intrinsic"][1][0], cam2["Intrinsic"][1][1], cam2["Intrinsic"][1][2], //
                                            cam2["Intrinsic"][2][0], cam2["Intrinsic"][2][1], cam2["Intrinsic"][2][2]});
        cams_[1].image_size = cv::Size_<int>(cam2["SensorSize"]["Width"], cam2["SensorSize"]["Height"]);
    }

    void VxsSensorPublisher::SensorPublishingLoop()
    {
        while (!flag_shutdown_request_)
        {
            RawSensorFrame frame_data;
            {
                std::unique_lock<std::mutex> lock(frame_queue_mutex_);
                frame_queue_cv_.wait(lock, [this]
                                     { return !frame_queue_.empty() || flag_shutdown_request_; });

                if (frame_queue_.empty())
                    continue;

                frame_data = frame_queue_.front();
                frame_queue_.pop();

                if (publish_events_) // streaming based publishing
                {
                    vxsdk::vxXYZT *eventsXYZT = (vxsdk::vxXYZT *)&(*frame_data.data)[0];
                    if (frame_data.N > 0)
                    {
                        PublishStampedPointcloud(frame_data.num_entries, eventsXYZT, frame_data.stamp);
                    }
                }
                else // Frame based data
                {
                    // Get data from the sensor
                    float *frameXYZ = (float *)&(*frame_data.data)[0];
                    // Extract frame
                    std::vector<cv::Vec3f> points;

                    cv::Mat frame = UnpackFrameSensorData(frameXYZ, points);
                    // Publish sensor data as a depth image
                    if (publish_depth_image_)
                    {
                        PublishDepthImage(frame, frame_data.stamp);
                    }
                    if (publish_pointcloud_)
                    {
                        PublishPointcloud(points, frame_data.stamp);
                    }
                }
            }
        }
    }

    void VxsSensorPublisher::RGBPublishingLoop()
    {
        while (!flag_shutdown_request_)
        {
            RGBFrame rgb_data;
            {
                std::unique_lock<std::mutex> lock(rgb_queue_mutex_);
                rgb_queue_cv_.wait(lock, [this]
                                   { return !rgb_queue_.empty() || flag_shutdown_request_; });

                if (rgb_queue_.empty())
                    continue;

                rgb_data = rgb_queue_.front();
                rgb_queue_.pop();

                PublishGrayscaleImage(rgb_data.img, rgb_data.stamp);
            }
        }
    }


    void VxsSensorPublisher::PublishGrayscaleImage(const cv::Mat &rgb_img, const ros::Time &stamp)
    {
        std_msgs::Header header;
        header.stamp = stamp;
        header.frame_id = "mono";

        cv::Mat gray_img;
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);

        gray_publisher_->publish(cv_bridge::CvImage(header, "mono8", gray_img).toImageMsg());
    }

    void VxsSensorPublisher::PublishDepthImage(const cv::Mat &depth_image, const ros::Time &stamp)
    {
        cv_bridge::CvImage cv_image(              //
            std_msgs::Header(),                   //
            sensor_msgs::image_encodings::MONO16, //
            depth_image);
        sensor_msgs::Image depth_image_msg = *cv_image.toImageMsg();
        // depth_image_msg.header.stamp = ros::Time::now();
        depth_image_msg.header.stamp = stamp;
        depth_image_msg.header.frame_id = "sensor";
        depth_image_msg.height = depth_image.rows;
        depth_image_msg.width = depth_image.cols;
        depth_image_msg.encoding = sensor_msgs::image_encodings::MONO16;

        // Create camera info message
        sensor_msgs::CameraInfo cam_info_msg;

        cam_info_msg.header.stamp = depth_image_msg.header.stamp;

        cam_info_msg.header.frame_id = depth_image_msg.header.frame_id;
        cam_info_msg.width = depth_image_msg.width;
        cam_info_msg.height = depth_image_msg.height;
        cam_info_msg.distortion_model = "plumb_bob";

        cam_info_msg.D = {cams_[0].dist[0], cams_[0].dist[1], cams_[0].dist[2], cams_[0].dist[3], cams_[0].dist[4]};
        cam_info_msg.K = {                                                      //
                          cams_[0].K(0, 0), cams_[0].K(0, 1), cams_[0].K(0, 2), //
                          cams_[0].K(1, 0), cams_[0].K(1, 1), cams_[0].K(1, 2), //
                          cams_[0].K(2, 0), cams_[0].K(2, 1), cams_[0].K(2, 2)};
        cam_info_msg.R = {                                                      //
                          cams_[0].R(0, 0), cams_[0].R(0, 1), cams_[0].R(0, 2), //
                          cams_[0].R(1, 0), cams_[0].R(1, 1), cams_[0].R(1, 2), //
                          cams_[0].R(2, 0), cams_[0].R(2, 1), cams_[0].R(2, 2)};

        cam_info_msg.P = {                                                         //
                          cams_[0].K(0, 0), cams_[0].K(0, 1), cams_[0].K(0, 2), 0, //
                          cams_[0].K(1, 0), cams_[0].K(1, 1), cams_[0].K(1, 2), 0, //
                          cams_[0].K(2, 0), cams_[0].K(2, 1), cams_[0].K(2, 2)};
        // publish depth image and camera info
        depth_publisher_->publish(depth_image_msg);
        cam_info_publisher_->publish(cam_info_msg);
    }

    void VxsSensorPublisher::PublishPointcloud(const std::vector<cv::Vec3f> &points, const ros::Time &stamp)
    {
        sensor_msgs::PointCloud2 msg;

        // Set the header
        // msg.header.stamp = ros::Time::now();
        msg.header.stamp = stamp;
        msg.header.frame_id = "sensor";

        // Unordered pointcloud. Height is 1 and Width is the size (N)
        const size_t N = points.size();
        msg.height = 1;
        msg.width = N;

        // Define the point cloud fields
        sensor_msgs::PointField x, y, z;
        x.name = "x";
        x.offset = 0;
        x.datatype = sensor_msgs::PointField::FLOAT32;
        x.count = 1;
        y.name = "y";
        y.offset = 4;
        y.datatype = sensor_msgs::PointField::FLOAT32;
        y.count = 1;
        z.name = "z";
        z.offset = 8;
        z.datatype = sensor_msgs::PointField::FLOAT32;
        z.count = 1;

        msg.fields.push_back(x);
        msg.fields.push_back(y);
        msg.fields.push_back(z);

        msg.point_step = 12; // Size of a point in bytes
        msg.row_step = msg.point_step * msg.width;

        // Allocate memory for the point cloud data
        msg.data.resize(msg.row_step * msg.height);

        // Populate the point cloud data
        uint8_t *ptr = &msg.data[0];
        for (size_t i = 0; i < msg.width; ++i)
        {
            float *point = reinterpret_cast<float *>(ptr);
            point[0] = points[i][0]; // X coordinate
            point[1] = points[i][1]; // Y coordinate
            point[2] = points[i][2]; // Z coordinate
            ptr += msg.point_step;
        }
        pcloud_publisher_->publish(msg);
    }

    void VxsSensorPublisher::PublishStampedPointcloud(const int N, vxsdk::vxXYZT *eventsXYZT, const ros::Time &cloud_stamp)
    {
        if (N <= 0 || eventsXYZT == nullptr)
            return;

        sensor_msgs::PointCloud2 msg;

        msg.header.stamp = cloud_stamp;
        msg.header.frame_id = "sensor";

        // Unordered pointcloud. Height is 1 and Width is the size (N)
        msg.height = 1;
        msg.width = N;

        // Define the point cloud fields
        sensor_msgs::PointField x, y, z, t;
        x.name = "x";
        x.offset = 0;
        x.datatype = sensor_msgs::PointField::FLOAT32;
        x.count = 1;
        y.name = "y";
        y.offset = 4;
        y.datatype = sensor_msgs::PointField::FLOAT32;
        y.count = 1;
        z.name = "z";
        z.offset = 8;
        z.datatype = sensor_msgs::PointField::FLOAT32;
        z.count = 1;
        t.name = "t";
        t.offset = 12;
        t.datatype = sensor_msgs::PointField::FLOAT64;
        t.count = 1;

        msg.fields.push_back(x);
        msg.fields.push_back(y);
        msg.fields.push_back(z);
        msg.fields.push_back(t);

        msg.point_step = sizeof(float) * 3 + sizeof(double); // Size of a point in bytes
        msg.row_step = msg.point_step * msg.width;

        // Allocate memory for the point cloud data
        msg.data.resize(msg.row_step * msg.height);

        // Populate the point cloud data
        uint8_t *ptr = &msg.data[0];
        const double ref_stamp = eventsXYZT[0].timestamp * PERIOD_75_MHZ;
        for (size_t i = 0; i < msg.width; ++i)
        {
            float *point = reinterpret_cast<float *>(ptr);
            point[0] = eventsXYZT[i].x * 1e-3f;                                                                                      // X coordinate (mm -> m)
            point[1] = eventsXYZT[i].y * 1e-3f;                                                                                      // Y coordinate (mm -> m)
            point[2] = eventsXYZT[i].z * 1e-3f;                                                                                      // Z coordinate (mm -> m)
            *reinterpret_cast<double *>(ptr + t.offset) = eventsXYZT[i].timestamp * PERIOD_75_MHZ - ref_stamp + cloud_stamp.toSec(); // relative to ros message stamp
            ptr += msg.point_step;
        }
        evcloud_publisher_->publish(msg);
    }

    void VxsSensorPublisher::PublishIMUSample(const imu::IMUSample &sample, const ros::Time &stamp)
    {
        // Drop corrupted frames: accel ADC saturation (±16g = ±156.96 m/s²) or gyro at full FSR
        constexpr float ACCEL_CLAMP = 150.0f;
        constexpr float GYRO_CLAMP  = 4.2f; // 240 deg/s × π/180
        if (std::abs(sample.aX) > ACCEL_CLAMP || std::abs(sample.aY) > ACCEL_CLAMP ||
            std::abs(sample.aZ) > ACCEL_CLAMP || std::abs(sample.omegaX) > GYRO_CLAMP ||
            std::abs(sample.omegaY) > GYRO_CLAMP || std::abs(sample.omegaZ) > GYRO_CLAMP)
            return;

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "IMU";

        //@TODO: Find covariance values from IMU manufacturer
        imu_msg.orientation_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        imu_msg.angular_velocity_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        imu_msg.linear_acceleration_covariance = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        // !TODO: Assign calibrated orientation if necessary
        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1;

        imu_msg.angular_velocity.x = sample.omegaX;
        imu_msg.angular_velocity.y = sample.omegaY;
        imu_msg.angular_velocity.z = sample.omegaZ;

        imu_msg.linear_acceleration.x = sample.aX;
        imu_msg.linear_acceleration.y = sample.aY;
        imu_msg.linear_acceleration.z = sample.aZ;

        imu_publisher_->publish(imu_msg);
    }

    bool VxsSensorPublisher::UpdateObservationWindowCB(
        vxs_sensor_ros1::UpdateObservationWindow::Request &request,
        vxs_sensor_ros1::UpdateObservationWindow::Response &result)
    {
        if (request.on_time > 1000)
        {
            result.status_message = "Parameter on_time greater than 1000!";
            result.success = false;
        }
        else if (request.on_time < 0 || request.period_time < 0)
        {
            result.status_message = "Parameters on_time and period_time must be non-negative!";
            result.success = false;
        }
        else if (request.period_time < request.on_time)
        {
            result.status_message = "Parameter period_time must be greater than on_time!";
            result.success = false;
        }
        else
        {
            on_time_ = request.on_time;
            period_time_ = request.period_time;
            result.status_message = "vxs_node: Updating observation window...";
            result.success = true;
        }
        flag_update_observation_window_ = true;
        return true;
    }

} // end namespace vxs_ros