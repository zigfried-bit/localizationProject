/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 15:56:11
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 16:10:23
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/publisher/cloud_publisher.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class CloudPublisher {
    public:
        CloudPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, size_t buff_size);
        CloudPublisher() = default;

        void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time);
        void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);

        bool HasSubscribers();

    private:
        void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time);
    
    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
};
}
#endif