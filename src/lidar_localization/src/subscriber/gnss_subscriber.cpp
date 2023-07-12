/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 10:40:25
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 11:19:12
 * @FilePath: /localizationProject/src/lidar_localization/src/subscriber/gnss_subscriber.cpp
 * @Description: GNSS data ros subscriber interface definition
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */

/*
sensor_msgs/NavSatFix.msg

std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude

float64[9] position_covariance
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
uint8 position_covariance_type
*/
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
    buff_mutex_.lock();

    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);
    buff_mutex_.unlock();
}

// ParseData()的功能是从类中取数据
void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
    buff_mutex_.lock();

    if (new_gnss_data_.size() > 0) {
        // void insert(iterator pos, InputIterator first,InputIterator last);
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
    buff_mutex_.unlock();
}
}