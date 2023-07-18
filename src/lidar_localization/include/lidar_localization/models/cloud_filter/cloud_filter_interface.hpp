/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-17 10:38:19
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-17 10:55:06
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/models/cloud_filter/cloud_filter_interface.hpp
 * @Description: 点云滤波模块的接口
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class CloudFilterInterface {
    public:
        virtual ~CloudFilterInterface() = default;

        virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
        // Filter()是纯虚函数
        // 只是在基类中抽象出一个方法，且该基类只能被继承，而不能被体化instantiation
        // 这个方法的实现必须在派生类中被实现
};
}
#endif