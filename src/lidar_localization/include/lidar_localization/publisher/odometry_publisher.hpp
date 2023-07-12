/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 16:55:35
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 17:07:06
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/publisher/odometry_publisher.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace lidar_localization {
class OdometryPublisher {
    public:
        OdometryPublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          int buff_size);
        OdometryPublisher() = default;

        void Publish(const Eigen::Matrix4f& transform_matrix, double time);
        void Publish(const Eigen::Matrix4f& transform_matrix);

        bool HasSubscribers();
    
    private:
        void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);
    
    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;
};
}
#endif