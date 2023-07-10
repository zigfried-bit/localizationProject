/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-10 10:32:07
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-10 11:09:21
 * @FilePath: /localizationProject/src/lidar_localization/src/sensor_data/gnss_data.cpp
 * @Description: GNSS data definition
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */

#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"

// 静态成员变量必须在类外初始化
double lidar_localization::GNSSData::origin_longitude = 0.0;
double lidar_localization::GNSSData::origin_latitude = 0.0;
double lidar_localization::GNSSData::origin_altitude = 0.0;
bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {
void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
}
}