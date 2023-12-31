/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-17 10:59:30
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-17 20:00:48
 * @FilePath: /localizationProject/src/lidar_localization/src/model/cloud_filter/voxel_filter.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();

    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    
    LOG(INFO) << "Voxel Filter 的参数为：" << std::endl
            << leaf_size_x << ", "
            << leaf_size_y << ", "
            << leaf_size_z 
            << std::endl << std::endl;

    return true;
}

bool VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}

}