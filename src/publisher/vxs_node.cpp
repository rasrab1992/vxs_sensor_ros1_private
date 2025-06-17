#include <chrono>
#include <functional>

#include "publisher/vxs_node.hpp"
#include "common.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace vxs_ros1
{
    const float FilteringParams::DEFAULT_PREFILTERING_THRESH = 2.0;
    const float FilteringParams::DEFAULT_FILTERP1 = 0.1;

    VxsSensorPublisher::VxsSensorPublisher(const ros::NodeHandle &nh,
                                           const ros::NodeHandle &nhp) : nh_(nh),                        //
                                                                         nhp_(nhp),                      //
                                                                         frame_polling_thread_(nullptr), //
                                                                         depth_publisher_(nullptr),      //
                                                                         cam_info_publisher_(nullptr),   //
                                                                         pcloud_publisher_(nullptr),     //
                                                                         evcloud_publisher_(nullptr),    //
                                                                         flag_shutdown_request_(false)
    {
        std::string package_directory = ros::package::getPath("vxs_sensor_ros1");
        ROS_INFO_STREAM("Package directory: " << package_directory);
        // Declare & Get parameters
        nhp.param<bool>("publish_depth_image", publish_depth_image_, false);
        nhp.param<bool>("publish_pcloud", publish_pointcloud_, false);
        nhp.param<bool>("publish_events", publish_events_, false);
        nhp.param<int>("fps", fps_, true);
        period_ = std::lround(1000.0f / fps_); // period in ms (will be used in initialization if streaming events)

        if ((publish_depth_image_ || publish_pointcloud_) && publish_events_)
        {
            publish_events_ = false;
            ROS_INFO_STREAM("Both frame based mode (publishe_depth_image or publish_pcloud) and streaming mode (publish_events_) specificied. Disabling streaming mode.");
        }
        if (!publish_depth_image_ && !publish_pointcloud_ && !publish_events_)
        {
            publish_depth_image_ = true;
            ROS_INFO_STREAM("No publishing mode (frame-based or streaming) specified. Switching to frame-based (publish_depth_image).");
        }

        nhp.param<std::string>("config_json", config_json_, "config/and2_median_golden.json");
        nhp.param<std::string>("calib_json", calib_json_, "config/default_calib.json");

        // Filtering parameters
        nhp.param<int>("binning_amount", filtering_params_.binning_amount, FilteringParams::DEFAULT_BINNING);
        nhp.param<float>("prefiltering_threshold", filtering_params_.prefiltering_threshold, FilteringParams::DEFAULT_PREFILTERING_THRESH);
        nhp.param<float>("filterP1", filtering_params_.filterP1, FilteringParams::DEFAULT_FILTERP1);
        nhp.param<int>("temporal_threshold", filtering_params_.temporal_threshold, FilteringParams::DEFAULT_TEMPORAL_THRESH);
        nhp.param<int>("spatial_threshold", filtering_params_.spatial_threshold, FilteringParams::DEFAULT_SPATIAL_THRESH);

        // Print param values
        ROS_INFO_STREAM("Publish frame-based depth image (publlish_depth_image): " << (publish_depth_image_ ? "YES." : "NO."));
        ROS_INFO_STREAM("Publish frame-based pointcloud (publish_pcloud): " << (publish_pointcloud_ ? "YES." : "NO."));
        ROS_INFO_STREAM("Publish streaming based (stamped) point cloud (publish_events): " << (publish_events_ ? "YES." : "NO."));
        ROS_INFO_STREAM("FPS: " << fps_ << " and period in ms: " << period_);
        ROS_INFO_STREAM("Config JSON: " << config_json_);
        ROS_INFO_STREAM("Calibration JSON: " << calib_json_);

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

        evcloud_publisher_ = publish_events_ ? std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::PointCloud2>("pcloud/events", 10)) : nullptr;

        cam_info_publisher_ = std::make_shared<ros::Publisher>(nhp_.advertise<sensor_msgs::CameraInfo>("sensor/camera_info", 10));
        // Initialize & start polling thread
        ROS_INFO_STREAM("Starting publisher thread...");
        frame_polling_thread_ = std::make_shared<std::thread>(std::bind(&VxsSensorPublisher::FramePollingLoop, this));
        ROS_INFO_STREAM("Done!");
    }

    VxsSensorPublisher::~VxsSensorPublisher()
    {
        flag_shutdown_request_ = true;
        if (frame_polling_thread_)
        {
            if (frame_polling_thread_->joinable())
            {
                frame_polling_thread_->join();
            }
        }
        frame_polling_thread_ = nullptr;
        vxsdk::vxStopSystem();
    }

    bool VxsSensorPublisher::InitSensor()
    {
        // Set the frame rate (or time window)
        vxsdk::pipelineType pipeline_type;
        if (publish_events_)
        {
            pipeline_type = vxsdk::pipelineType::all; // Get everything out XYT-XYT pairs and XYZT
            vxsdk::vxSetStreamingDuration(period_);
        }
        else
        {
            pipeline_type = vxsdk::pipelineType::fbPointcloud;
            vxsdk::vxSetFPS(fps_);
        }
        // Set filtering parameters
        vxsdk::vxSetBinningAmount(filtering_params_.binning_amount);
        vxsdk::vxSetFilteringParameters(              //
            filtering_params_.prefiltering_threshold, //
            filtering_params_.filterP1,               //
            filtering_params_.temporal_threshold,     //
            filtering_params_.spatial_threshold);

        // Start the SDK Engine.
        int cam_num = vxsdk::vxStartSystem( //
            config_json_.c_str(),           //
            calib_json_.c_str(),            //
            pipeline_type);

        return cam_num > 0;
    }

    void VxsSensorPublisher::FramePollingLoop()
    {
        flag_in_polling_loop_ = true;
        int counter = 0;
        while (!flag_shutdown_request_)
        {
            // Wait until data ready
            while (!vxsdk::vxCheckForData())
            {
            }
            if (publish_events_) // streaming based publishing
            {
                int N;
                vxsdk::vxXYZT *eventsXYZT = vxsdk::vxGetXYZT(N);
                PublishStampedPointcloud(N, eventsXYZT);
            }
            else // Frame based data
            {
                // Get data from the sensor
                float *frameXYZ = vxsdk::vxGetFrameXYZ();
                counter++;
                // Extract frame
                std::vector<cv::Vec3f> points;

                cv::Mat frame = UnpackFrameSensorData(frameXYZ, points);
                //   Publish sensor data as a depth image
                if (publish_depth_image_)
                {
                    PublishDepthImage(frame);
                }
                if (publish_pointcloud_)
                {
                    PublishPointcloud(points);
                }
            }
        }
        flag_in_polling_loop_ = false;
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

    void VxsSensorPublisher::PublishDepthImage(const cv::Mat &depth_image)
    {
        cv_bridge::CvImage cv_image(              //
            std_msgs::Header(),                   //
            sensor_msgs::image_encodings::MONO16, //
            depth_image);
        sensor_msgs::Image depth_image_msg = *cv_image.toImageMsg();
        depth_image_msg.header.stamp = ros::Time::now();
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
        cam_info_msg.distortion_model = "plumn_bob";

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

    void VxsSensorPublisher::PublishPointcloud(const std::vector<cv::Vec3f> &points)
    {
        sensor_msgs::PointCloud2 msg;

        // Set the header
        msg.header.stamp = ros::Time::now();
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

    void VxsSensorPublisher::PublishStampedPointcloud(const int N, vxsdk::vxXYZT *eventsXYZT)
    {
        sensor_msgs::PointCloud2 msg;

        // Set the header
        msg.header.stamp = ros::Time::now();
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
        for (size_t i = 0; i < msg.width; ++i)
        {
            float *point = reinterpret_cast<float *>(ptr);
            point[0] = eventsXYZT[i].x; // X coordinate
            point[1] = eventsXYZT[i].y; // Y coordinate
            point[2] = eventsXYZT[i].z; // Z coordinate
            *(double *)(ptr + t.offset) = *(double *)&(eventsXYZT[i].timestamp);
            ptr += msg.point_step;
        }
        evcloud_publisher_->publish(msg);
    }

} // end namespace vxs_ros