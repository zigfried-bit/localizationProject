/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 17:07:43
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 17:17:24
 * @FilePath: /localizationProject/src/lidar_localization/src/publisher/odometry_publisher.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     int buff_size)
    :nh_(nh) {
    
    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix, double time) {
    ros::Time ros_time((float)time);
    PublishData(transform_matrix, ros_time);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
    ros::Time time = ros::Time::now();
    PublishData(transform_matrix, time);
}

void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time) {
    odometry_.header.stamp = time;

    //set the position
    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    // transform矩阵转四元数
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
}

bool OdometryPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}

}