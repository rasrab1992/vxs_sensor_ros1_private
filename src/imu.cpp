#include "imu.hpp"

#include "SDK2.h"

#include <iostream>

namespace imu
{
    const double DEG2RAD = M_PI / 180.0;

    IMUSample LinearInterpolation(const double &t, const IMUSample &sample1, const IMUSample &sample2)
    {

        IMUSample sample_t;
        sample_t.stamp_seconds = t;

        const double scaler = (t - sample1.stamp_seconds) / (sample2.stamp_seconds - sample1.stamp_seconds);
        sample_t.aX = sample1.aX + scaler * (sample2.aX - sample1.aX);
        sample_t.aY = sample1.aY + scaler * (sample2.aY - sample1.aY);
        sample_t.aZ = sample1.aZ + scaler * (sample2.aZ - sample1.aZ);

        sample_t.omegaX = sample1.omegaX + scaler * (sample2.omegaX - sample1.omegaX);
        sample_t.omegaY = sample1.omegaY + scaler * (sample2.omegaY - sample1.omegaY);
        sample_t.omegaZ = sample1.omegaZ + scaler * (sample2.omegaZ - sample1.omegaZ);

        return sample_t;
    }

    IMUSample &IMUSample::operator=(const vxsdk::vxIMU &vx_imu_data)
    {
        accelX = vx_imu_data.accelX;
        accelY = vx_imu_data.accelY;
        accelZ = vx_imu_data.accelZ;
        gyroX = vx_imu_data.gyroX;
        gyroY = vx_imu_data.gyroY;
        gyroZ = vx_imu_data.gyroZ;
        aX = accelX * accel_scaler;
        aY = accelY * accel_scaler;
        aZ = accelZ * accel_scaler;
        stamp = vx_imu_data.timestamp;
        omegaX = gyroX * gyro_scaler;
        omegaY = gyroY * gyro_scaler;
        omegaZ = gyroZ * gyro_scaler;
        stamp_seconds = stamp * PERIOD_75_MHZ; // conversion to absolute seconds

        return *this;
    }

    std::ostream &operator<<(std::ostream &os, const IMUSample &sample)
    {
        os << std::to_string(sample.accelX) << ", " << std::to_string(sample.accelY) << ", " //
           << std::to_string(sample.accelZ) << ", " << std::to_string(sample.gyroX) << ", "  //
           << std::to_string(sample.gyroY) << ", " << std::to_string(sample.gyroZ) << " | "  //
           << std::to_string(sample.aX) << ", " << std::to_string(sample.aY) << ", "         //
           << std::to_string(sample.aZ) << ", " << std::to_string(sample.omegaX) << ", "     //
           << std::to_string(sample.omegaY) << ", " << std::to_string(sample.omegaZ);
        return os;
    }
} // namespace imu