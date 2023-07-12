/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 15:12:52
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 15:15:21
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/subscriber/imu_subscriber.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
class IMUSubscriber {
    public:
        IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        IMUSubscriber() = default;
        void ParseData(std::deque<IMUData>& deque_imu_data);
    
    private:
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<IMUData> new_imu_data_;

        std::mutex buff_mutex_; 
};
}
#endif