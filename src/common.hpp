
/**
 * @brief Common structs for both publisher and subscriber
 *
 */

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <SDK2.h>

namespace vxs_ros
{
    struct CameraCalibration
    {
        cv::Vec<float, 5> dist = {0, 0, 0, 0, 0};
        cv::Vec3f t = {0, 0, 0};
        cv::Matx<float, 3, 3> R = cv::Matx<float, 3, 3>::eye();
        cv::Matx<float, 3, 3> K = cv::Matx<float, 3, 3>::eye();
        cv::Matx<float, 3, 4> P = cv::Matx<float, 3, 4>::eye();
        cv::Size_<int> image_size = cv::Size_<int>(0, 0);
    };

}

#endif
