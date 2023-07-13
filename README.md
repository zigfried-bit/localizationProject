<!--
 * @Author: zigfried 3572931733@qq.com
 * @Date: 2023-07-09 20:25:02
 * @LastEditors: zigfried 3572931733@qq.com
 * @LastEditTime: 2023-07-13 16:22:22
 * @FilePath: /localizationProject/README.md
 * @Description: 
 * 
 * Copyright (c) 2023 by zigfried, All Rights Reserved. 
-->
# localizationProject
先从GNSSData开始看的，GNSS数据的ROS接口，大概需要以下4个文件：
- gnss_data.hpp
- gnss_data.cpp
- gnss_subscriber.hpp
- gnss_subscriber.cpp

```cpp .line-numbers
// 原始代码
// localization_in_auto_driving/lidar_localization/src/sensor_data/gnss_data.cpp
bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;
    // 下面是线性差值操作，略
}
```
**有几个小bug**
1. 当同步时间距离两端差值大于0.2时，只是break的话，还是有可能执行下面的线性插值操作，应该直接return false；
2. 其次，也不应该都pop_front()，与队列首元素差值大，说明首元素对线性差值没有价值了，可以删除首元素；与第二个元素差值大，不能说明第一个没用了呀，可能下一次就可以用上了。
3. 原来这样写，之所以没问题，很可能是因为，绝大多数情况下，当同步时间找到区间后，没有出现数据丢失导致时间差值过大的情况。
4. 0.2这个差值是人为设定的，应该要根据传感器的实际情况调整。
5. [知乎也有评论在讨论上面说的问题](https://zhuanlan.zhihu.com/p/108853312)

接下来看imu，cloud，publisher
tf可以考虑升级一下tf2

小细节：global_defination.cmake
这个definition拼错了

参照tag4.0添加前端里程计时遇到"multiple definition of `main';"
原因是CMakeLists.txt中使用了`file(GLOB_RECURSE ALL_SRCS "*.cpp")`
如果编译生成2个及以上可执行文件就会报错。
最简单的办法是：将另外一个暂时用不到的文件删掉即可