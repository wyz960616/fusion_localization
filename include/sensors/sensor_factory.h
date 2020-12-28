//@Description
//@Author:wyz
//@Time:2020/12/28 上午9:32                            

#ifndef LIDAR_LOCALIZATION_SENSORFACTORY_H
#define LIDAR_LOCALIZATION_SENSORFACTORY_H
#include "sensors/imu.h"
#include "sensors/laser_scan.h"
#include "sensors/odom.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <memory>


//为了将ros和主要逻辑代码分开
namespace fusion_localization {
class SensorFactory {
public:
    static ImuPtr CreateImuPtr(const sensor_msgs::ImuConstPtr &imu_msg);
    static LaserScanPtr CreateLaserScanPtr(const sensor_msgs::LaserScanConstPtr &laser_scan_msg);
    static OdomPtr CreateOdomPtr(const nav_msgs::OdometryConstPtr &odom_msg);

    static void OdomToRosMsg(const OdomPtr &odom, nav_msgs::Odometry &odom_ros);
    static void OdomToRosMsg(const OdomPtr &odom, nav_msgs::Odometry &odom_ros,
                         const std::string& frame_id, const std::string &child_frame_id);
    static void LaserScanToRosMsg(const LaserScanPtr &laser, sensor_msgs::LaserScan &laser_ros);
    static void ImuToRosMsg(const ImuPtr &imu, sensor_msgs::Imu &imu_ros);
public:
    static SensorFactory& GetSensorFactoryInstance();
private:
    SensorFactory() = default;
    ~SensorFactory() = default;
    const SensorFactory& operator = (const SensorFactory& );
};

}


#endif //LIDAR_LOCALIZATION_SENSORFACTORY_H
