/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 16:38:14
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 16:47:19
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/tf_listener/tf_listener.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_TF_LISTENER_HPP_
#define LIDAR_LOCALIZATION_TF_LISTENER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace lidar_localization {
class TFListener {
    public:
        TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
        TFListener() = default;

        bool LookupData(Eigen::Matrix4f& transform_matrix);
    
    private:
        bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);
    
    private:
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        std::string base_frame_id_;
        std::string child_frame_id_;
};
}
#endif