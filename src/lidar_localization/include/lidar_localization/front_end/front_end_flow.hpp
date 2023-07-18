/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-17 20:29:52
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-17 20:35:33
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/front_end/front_end_flow.hpp
 * @Description: front end 任务管理
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/front_end/front_end.hpp"

namespace lidar_localization {
class FrontEndFlow {
    public:
        FrontEndFlow(ros::NodeHandle& nh);

        bool Run();
        bool SaveMap();
        bool PublishGlobalMap();

    private:
        bool ReadData();
        bool InitCalibration();
        bool InitGNSS();
        bool HasData();
        bool ValidData();
        bool UpdateGNSSOdometry();
        bool UpdateLaserOdometry();
        bool PublishData();
    
    private:
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
        std::shared_ptr<TFListener> lidar_to_imu_ptr_;
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
        std::shared_ptr<FrontEnd> front_end_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<IMUData> imu_data_buff_;
        std::deque<GNSSData> gnss_data_buff_;
        Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
        CloudData current_cloud_data_;
        IMUData current_imu_data_;
        GNSSData current_gnss_data_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;
        Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}
#endif