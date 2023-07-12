/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-10 10:32:07
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-11 10:38:04
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

    origin_longitude = longitude;
    origin_latitude = latitude;
    origin_altitude = altitude;

    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);  
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    while (UnsyncedData.size() >= 2) {
        // 若UnsyncedData第一个元素时间大于同步时间，说明整个队列都晚于同步时间，认为同步失败
        if (UnsyncedData.front().time > sync_time)
            return false;
        // 若UnsyncedData第二个元素时间小于同步时间，说明同步时间的位置一定在第二个元素后，删除队列首元素，continue
        // 这样操作，只要同步时间晚于队列首元素，最后一定可以将同步时间限制在队列第一，第二元素之间
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        // 若同步时间比首元素晚0.2s，时间差值较大，说明数据有丢失，删除首元素，同步失败
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        // 若同步时间比第二个元素早0.2s，时间差值较大，说明数据有丢失，同步失败
        if (UnsyncedData.at(1).time - sync_time > 0.2)
            return false;
        
        break;
    }
    // 若队列被删得只剩1个元素，同步失败
    if (UnsyncedData.size() < 2)
        return false;
    
    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    // 线性差值，和loam中的imu插值方法如出一辙
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude  = front_data.latitude  * front_scale + back_data.latitude  * back_scale;
    synced_data.altitude  = front_data.altitude  * front_scale + back_data.altitude  * back_scale;
    synced_data.local_E   = front_data.local_E   * front_scale + back_data.local_E   * back_scale;
    synced_data.local_N   = front_data.local_N   * front_scale + back_data.local_N   * back_scale;
    synced_data.local_U   = front_data.local_U   * front_scale + back_data.local_U   * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}
}