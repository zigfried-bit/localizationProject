/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-11 15:23:22
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 15:50:41
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/sensor_data/cloud_data.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization {
class CloudData {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;
    
    public:
        CloudData()
            :cloud_ptr(new CLOUD()) {
        }
    
    public:
        double time;
        CLOUD_PTR cloud_ptr;

};
}
#endif