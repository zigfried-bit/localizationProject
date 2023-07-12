/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 17:22:36
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 17:22:59
 * @FilePath: /localizationProject/src/lidar_localization/src/subscriber/cloud_subscriber.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    buff_mutex_.lock();
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    new_cloud_data_.push_back(cloud_data);
    buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    buff_mutex_.lock();
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
    buff_mutex_.unlock();
}
}