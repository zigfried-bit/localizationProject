/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-17 11:33:12
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-17 15:20:13
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/models/registration/registration_interface.hpp
 * @Description: 点云配准模块接口
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class RegistrationInterface {
    public: 
        virtual ~RegistrationInterface() = default;

        virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
        virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                               const Eigen::Matrix4f& predict_pose,
                               CloudData::CLOUD_PTR& result_cloud_ptr,
                               Eigen::Matrix4f& result_pose) = 0;
};
}
#endif