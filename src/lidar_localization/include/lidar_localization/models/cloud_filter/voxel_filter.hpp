/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-17 10:46:34
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-17 10:58:10
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/models/cloud_filter/voxel_filter.hpp
 * @Description: 体素滤波模块
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class VoxelFilter: public CloudFilterInterface {
    public:
        VoxelFilter(const YAML::Node& node);
        VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
    
    private:
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    private:
        pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};
}
#endif