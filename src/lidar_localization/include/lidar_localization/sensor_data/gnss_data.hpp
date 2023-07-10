/*
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-10 09:37:18
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-10 10:56:13
 * @FilePath: /localizationProject/src/lidar_localization/include/lidar_localization/sensor_data/gnss_data.hpp
 * @Description: GNSS data declaration
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
 */
 
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

// 安装第三方库geographiclib https://github.com/geographiclib/geographiclib
#include <GeographicLib/LocalCartesian.hpp>

namespace lidar_localization {
class GNSSData {
    public:
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;
        int status = 0;
        int service = 0;

        static double origin_longitude;
        static double origin_latitude;
        static double origin_altitude;

    private:
        static GeographicLib::LocalCartesian geo_converter;
        static bool origin_position_inited;
    
    public:
        void InitOriginPosition();
        void UpdateXYZ();
        static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);

};
}
#endif