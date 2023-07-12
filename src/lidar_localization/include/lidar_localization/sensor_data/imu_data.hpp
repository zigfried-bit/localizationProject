/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 14:43:16
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 15:10:44
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/sensor_data/imu_data.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace lidar_localization {
class IMUData {
    public:
        struct LinearAcceleration {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct AngularVelocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        class Orientation {
            public:
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                double w = 0.0;
            
            public:
                void Normlize() {
                    double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                    x /= norm;
                    y /= norm;
                    z /= norm;
                    w /= norm;
                }
        };

        double time = 0.0;
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        Orientation orientation;
    
    public:
        Eigen::Matrix3f GetOrientationMatrix();
        static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};
}
#endif