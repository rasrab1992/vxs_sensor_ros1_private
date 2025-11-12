/**
 * @brief IMU structs and functions
 *
 */

#ifndef IMU_HPP_
#define IMU_HPP_

#include <stdlib.h>
#include <iostream>

namespace vxsdk
{
    struct vxIMU;
}

namespace imu
{
    extern const double DEG2RAD;

    struct IMUSample
    {
        //! For the following stetings, please visit,
        // https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf
        //
        //
        //! Accelerometer FSR (Full Scale Range in g) for 32767 (signed integer range)
        static const int16_t Accel_2g_FSR = 16384; // 32767 / 2
        static const int16_t Accel_4g_FSR = 8193;  // 32767 / 4
        static const int16_t Accel_16g_FSR = 2048; // 32767 / 16
        //
        //! Gyro FSR (Full Scale Range in  DPS (degrees/s) ) for 32767 (signed integer range)
        static const int16_t Gyro_2000_FSR = 2000;
        static const int16_t Gyro_1000_FSR = 1000;
        static const int16_t Gyro_500_FSR = 500;
        static const int16_t Gyro_250_FSR = 250;

        //! Period between time instances (corresponds to 75 Mhz)
        static constexpr double PERIOD_75_MHZ = 1e-9 * 13.3333;

        //! Using maximum FSR = 250 for gyro. @TODO: verify this!!!!)
        static constexpr float gyro_scaler = (1.0f * Gyro_250_FSR) / 32767;
        //! Accelerometer FSR = 16g (verified from readings).
        static constexpr float g = 9.81f;
        static constexpr float accel_scaler = g / Accel_16g_FSR;

        int16_t accelX;
        int16_t accelY;
        int16_t accelZ;
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;
        long long int stamp;
        //! Timestamp in seconds
        double stamp_seconds;

        float omegaX, omegaY, omegaZ;
        float aX, aY, aZ;

        inline IMUSample() : accelX(0), //
                             accelY(0), //
                             accelZ(0), //
                             gyroX(0),  //
                             gyroY(0),  //
                             gyroZ(0),  //
                             stamp(-1), //
                             omegaX(0), //
                             omegaY(0), //
                             omegaZ(0), //
                             aX(0),     //
                             aY(0),     //
                             aZ(0)
        {
        }

        inline IMUSample(
            const int16_t _accelX,                                        //
            const int16_t _accelY,                                        //
            const int16_t _accelZ,                                        //
            const int16_t _gyroX,                                         //
            const int16_t _gyroY,                                         //
            const int16_t _gyroZ,                                         //
            long long int _stamp) :                                       //
                                    accelX(_accelX),                      //
                                    accelY(_accelY),                      //
                                    accelZ(_accelZ),                      //
                                    gyroX(_gyroX),                        //
                                    gyroY(_gyroY),                        //
                                    gyroZ(_gyroZ),                        //
                                    stamp(_stamp),                        //
                                    stamp_seconds(stamp * PERIOD_75_MHZ), //
                                    omegaX(gyroX * gyro_scaler),          //
                                    omegaY(gyroY * gyro_scaler),          //
                                    omegaZ(gyroZ * gyro_scaler)

        {
        }

        //! Copy constructor
        inline IMUSample(const vxsdk::vxIMU &vx_sample)
        {
            *this = vx_sample;
        }

        //! Constructor with base stamp
        IMUSample(const vxsdk::vxIMU &vx_sample, const long long int &base_stamp);

        IMUSample &operator=(const vxsdk::vxIMU &vx_imu_data);
    };

    std::ostream &operator<<(std::ostream &os, const IMUSample &sample);

    /**
     * @brief Interpolate samples
     *
     * @param t Prescribed time
     * @param sample1 Sample #1
     * @param sample2 Sample #2
     * @return Interpolated sample (fields aX, aY, aZ, omegaX, omegaY, omegaZ)
     */
    IMUSample LinearInterpolation(const double &t, const IMUSample &sample1, const IMUSample &sample2);
} // namespace imu

#endif
