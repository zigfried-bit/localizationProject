/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 16:11:11
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 16:29:54
 * @FilePath: /localizationProject/src/lidar_localization/src/publisher/cloud_publisher.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time) {
    ros::Time ros_time((float)time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void CloudPublisher::PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}