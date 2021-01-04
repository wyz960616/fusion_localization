//@Description
//@Author:wyz
//@Time:2020/12/17 下午6:58
#ifndef LIDAR_LOCALIZATION_SENSOR_INTERFACE_H
#define LIDAR_LOCALIZATION_SENSOR_INTERFACE_H
#include <memory>
#include <string>
#include <deque>
#include "glog/logging.h"
namespace fusion_localization {
class Sensor;
typedef std::shared_ptr<Sensor> SensorPtr;
class Sensor {
public:
    explicit Sensor(double timestamp);
    /*!
     * @breif 生成指定时间戳的数据
     * @example 指定时间戳为 10,存在最近的两帧，时间戳为8 11，于是将该左右合成一个数据，完成对其
     * @param front_ptr 指定时间戳的前一帧
     * @param back_ptr 指定时间戳的后一帧
     * @param synced_timestamp 指定的时间戳的后一帧
     * @param median_ptr 完成校准后输出的数据
     * @return 对齐失败返回false
     */
    virtual bool SetMedianValue(const SensorPtr& front_ptr, const SensorPtr& back_ptr, double synced_timestamp,
                                       SensorPtr& median_ptr);
    /*!
     * @breif 根据没有对齐的数据队列，完成时间戳对齐操作
     * @param un_synced_data 未对齐的sensor数据队列
     * @param synced_timestamp 要对齐的时间戳
     * @param synced_data 对齐后的sensor数据队列
     * @return 对齐失败返回false
     */
    virtual bool SyncData(std::deque<SensorPtr>& un_synced_data, double synced_timestamp,
                          std::deque<SensorPtr>& synced_data);
    /*!
     * @return sensor的名称
     */
    virtual std::string Name();

    double timestamp_;

public:
    /*!
     * @breif 根据左右时间戳，得到前后数据的权重值，提供加权平均的ratio,注意要倒着取，越接近的ratio应当越大
     * @param front_timestamp
     * @param back_timestamp
     * @param synced_timestamp
     * @param front_scale 前侧数据的权重
     * @param back_scale  后侧数据的权重
     */
    static void MedianScale(double front_timestamp, double back_timestamp, double synced_timestamp,
                                 double& front_scale, double& back_scale) {
        front_scale = ( back_timestamp - synced_timestamp ) / ( back_timestamp - front_timestamp );
        back_scale = ( synced_timestamp - front_timestamp ) / ( back_timestamp - front_timestamp );
    }
};

}


#endif //LIDAR_LOCALIZATION_SENSOR_INTERFACE_H
