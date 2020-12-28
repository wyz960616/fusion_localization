//@Description
//@Author:wyz
//@Time:2020/12/17 下午8:19                            

#ifndef LIDAR_LOCALIZATION_IMU_H
#define LIDAR_LOCALIZATION_IMU_H
#include "sensors/sensor_interface.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace fusion_localization {
#define CAST_TO_IMU(x) std::static_pointer_cast<Imu>(x)
class Imu;
typedef std::shared_ptr<Imu> ImuPtr;

class Imu : public Sensor{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Imu(): Sensor(0){}
    Imu(double timestamp): Sensor(timestamp){}
    bool SetMedianValue(const SensorPtr& front_ptr, const SensorPtr& back_ptr, double synced_timestamp,
                                SensorPtr& median_ptr) override;
    std::string Name() override;
    Eigen::Matrix3d Rotation();

public:
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d linear_acceleration_;
    Eigen::Vector3d angular_velocity_;
};

}


#endif //LIDAR_LOCALIZATION_IMU_H
