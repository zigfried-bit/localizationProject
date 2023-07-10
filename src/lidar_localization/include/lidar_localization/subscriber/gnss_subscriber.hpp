/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-09 23:35:30
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-10 10:54:52
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/subscriber/gnss_subscriber.hpp
 * @Description: GNSS data ros subscriber interface declaration
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"

#include "lidar_localization/sensor_data/gnss_data.hpp"

namespace lidar_localization {
class GNSSSubscriber {
    public:
        GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        GNSSSubscriber() = default;
        void ParseData(std::deque<GNSSData>& deque_gnss_data);
    
    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<GNSSData> new_gnss_data_;

        std::mutex buff_mutex_;
};
}
#endif

