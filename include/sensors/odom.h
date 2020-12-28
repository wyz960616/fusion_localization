//@Description
//@Author:wyz
//@Time:2020/12/17 下午7:58                            

#ifndef LIDAR_LOCALIZATION_ODOM_H
#define LIDAR_LOCALIZATION_ODOM_H
#include "sensor_interface.h"
#include <Eigen/Core>
#include <Eigen/Dense>
namespace fusion_localization {
#define CAST_TO_ODOM(x) std::static_pointer_cast<Odom>(x)
class Odom;
typedef std::shared_ptr<Odom> OdomPtr;

class Odom : public Sensor{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Odom(): Sensor(0){}
    Odom(double timestamp): Sensor(timestamp) {}
    bool SetMedianValue(const SensorPtr &front_ptr, const SensorPtr &back_ptr, double synced_timestamp,
                                SensorPtr &median_ptr) override;
    std::string Name() override;
    Eigen::Matrix4d Pose();
public:
    Eigen::Quaterniond q_;
    Eigen::Vector3d t_;
};

}


#endif //LIDAR_LOCALIZATION_ODOM_H
