#include <chrono>
#include <functional>

#include "common.hpp"
#include "subscriber/vxs_subscriber.hpp"

#include <pcl_conversions/pcl_conversions.h> // For converting between ROS and PCL types
namespace vxs_ros
{
    VxsSensorSubscriber::VxsSensorSubscriber() : Node("vxs_cpp_subscriber"), cam_(nullptr)
    {
        cam_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>( //
            "/sensor/camera_info",                                                      //
            10,                                                                         //
            std::bind(&VxsSensorSubscriber::CameraInfoCB, this, std::placeholders::_1)  //
        );
        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(        //
            "/depth/image",                                                            //
            5,                                                                         //
            std::bind(&VxsSensorSubscriber::DepthImageCB, this, std::placeholders::_1) //
        );

        pcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( //
            "/pcloud/cloud",                                                           //
            5,                                                                         //
            std::bind(&VxsSensorSubscriber::PointcloudCB, this, std::placeholders::_1) //
        );

        evcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(       //
            "/pcloud/events",                                                                 //
            5,                                                                                //
            std::bind(&VxsSensorSubscriber::StampedPointcloudCB, this, std::placeholders::_1) //
        );
    }

    VxsSensorSubscriber::~VxsSensorSubscriber()
    {
    }

    void VxsSensorSubscriber::CameraInfoCB(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
    {
        if (!cam_)
        {
            cam_ = std::make_shared<CameraCalibration>();
            for (size_t r = 0; r < 3; r++)
            {
                for (size_t c = 0; c < 3; c++)
                {
                    cam_->K(r, c) = static_cast<float>(camera_info_msg->k[r * 3 + c]);
                    cam_->R(r, c) = static_cast<float>(camera_info_msg->r[r * 3 + c]);
                    cam_->P(r, c) = static_cast<float>(camera_info_msg->p[r * 4 + c]);
                }
                cam_->K(r, 3) = static_cast<float>(camera_info_msg->p[r * 4 + 3]);
            }

            for (size_t c = 0; c < 4; c++)
            {
                cam_->dist[c] = static_cast<float>(camera_info_msg->d[c]);
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Camera parameters acquired.");
        }
    }

    void VxsSensorSubscriber::DepthImageCB(const sensor_msgs::msg::Image::SharedPtr depth_img_msg)
    {
        try
        {
            cv::Mat depth_img = cv_bridge::toCvShare(depth_img_msg, sensor_msgs::image_encodings::MONO16)->image;
            cv::imshow("view", depth_img);
            cv::waitKey(10);
        }
        catch (const cv_bridge::Exception &e)
        {
            // auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'mono16'.", depth_img_msg->encoding.c_str());
        }
    }

    void VxsSensorSubscriber::PointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    {
        std::vector<cv::Vec3f> points;
        // const uint32_t row_step = pcl_msg->row_step;

        const uint32_t width = pcl_msg->width;
        const uint32_t height = pcl_msg->height;
        uint8_t *data_ptr = &pcl_msg->data[0];

        sensor_msgs::msg::PointField x_field = pcl_msg->fields[0];
        sensor_msgs::msg::PointField y_field = pcl_msg->fields[1];
        sensor_msgs::msg::PointField z_field = pcl_msg->fields[2];
        // Point field datatypes. Pointrcloud can be either FLOAT32 or FLOAT64
        // uint8 FLOAT32 = 7
        // uint8 FLOAT64 = 8
        const uint32_t field1_size = (x_field.datatype == 7 ? sizeof(float) : sizeof(double));
        const uint32_t field2_size = (y_field.datatype == 7 ? sizeof(float) : sizeof(double));
        const uint32_t field3_size = (z_field.datatype == 7 ? sizeof(float) : sizeof(double));
        const uint32_t size_of_fields = field1_size + field2_size + field3_size;

        for (size_t r = 0; r < height; r++)
        {
            for (size_t c = 0; c < width; c++)
            {
                const double x = x_field.datatype == 7 ? *(float *)(data_ptr + x_field.offset) : *(double *)(data_ptr + x_field.offset);
                const double y = y_field.datatype == 7 ? *(float *)(data_ptr + y_field.offset) : *(double *)(data_ptr + y_field.offset);
                const double z = z_field.datatype == 7 ? *(float *)(data_ptr + z_field.offset) : *(double *)(data_ptr + z_field.offset);

                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
                {
                    points.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
                    RCLCPP_INFO_STREAM(this->get_logger(), "Point: (" << x << ", " << y << ", " << z << ")");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid point read!");
                }
                data_ptr += size_of_fields;
            }
        }
    }

    void VxsSensorSubscriber::StampedPointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    {
        std::vector<vxsdk::vxXYZT> points;
        // const uint32_t row_step = pcl_msg->row_step;

        const uint32_t width = pcl_msg->width;
        const uint32_t height = pcl_msg->height;
        uint8_t *data_ptr = &pcl_msg->data[0];

        sensor_msgs::msg::PointField x_field = pcl_msg->fields[0];
        sensor_msgs::msg::PointField y_field = pcl_msg->fields[1];
        sensor_msgs::msg::PointField z_field = pcl_msg->fields[2];
        sensor_msgs::msg::PointField t_field = pcl_msg->fields[3];

        // Point field datatypes. Pint coords are either FLOAT32 or FLOAT64. Timestamp is FLOAT64 (which is converted to long long)
        const uint32_t field1_size = (x_field.datatype == 7 ? sizeof(float) : sizeof(double));
        const uint32_t field2_size = (y_field.datatype == 7 ? sizeof(float) : sizeof(double));
        const uint32_t field3_size = (z_field.datatype == 7 ? sizeof(float) : sizeof(double));
        const uint32_t field4_size = sizeof(double);
        const uint32_t size_of_fields = field1_size + field2_size + field3_size + field4_size;

        for (size_t r = 0; r < height; r++)
        {
            for (size_t c = 0; c < width; c++)
            {
                const double x = x_field.datatype == 7 ? *(float *)(data_ptr + x_field.offset) : *(double *)(data_ptr + x_field.offset);
                const double y = y_field.datatype == 7 ? *(float *)(data_ptr + y_field.offset) : *(double *)(data_ptr + y_field.offset);
                const double z = z_field.datatype == 7 ? *(float *)(data_ptr + z_field.offset) : *(double *)(data_ptr + z_field.offset);
                long long stamp = *(long long *)(data_ptr + t_field.offset);

                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
                {
                    points.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), stamp);
                    RCLCPP_INFO_STREAM(this->get_logger(), "Point (x, y, z, t): (" << x << ", " << y << ", " << z << ", " << stamp << ")");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid point read!");
                }
                data_ptr += size_of_fields;
            }
        }
    }

} // end namespace vxs_ros