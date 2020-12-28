//@Description
//@Author:wyz
//@Time:2020/12/28 上午9:32                            

#include "sensors/sensor_factory.h"

namespace fusion_localization {
SensorFactory &SensorFactory::GetSensorFactoryInstance() {
    static SensorFactory sensor_factory;
    return sensor_factory;
}

ImuPtr SensorFactory::CreateImuPtr(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuPtr imu_ptr = std::make_shared<Imu>(imu_msg->header.stamp.toSec());
    imu_ptr->orientation_.w() = imu_msg->orientation.w;
    imu_ptr->orientation_.x() = imu_msg->orientation.x;
    imu_ptr->orientation_.y() = imu_msg->orientation.y;
    imu_ptr->orientation_.z() = imu_msg->orientation.z;

    imu_ptr->linear_acceleration_.x() = imu_msg->linear_acceleration.x;
    imu_ptr->linear_acceleration_.y() = imu_msg->linear_acceleration.y;
    imu_ptr->linear_acceleration_.z() = imu_msg->linear_acceleration.z;

    imu_ptr->angular_velocity_.x() = imu_msg->angular_velocity.x;
    imu_ptr->angular_velocity_.y() = imu_msg->angular_velocity.y;
    imu_ptr->angular_velocity_.z() = imu_msg->angular_velocity.z;
    return imu_ptr;
}

LaserScanPtr SensorFactory::CreateLaserScanPtr(const sensor_msgs::LaserScanConstPtr &laser_scan_msg) {
    std::vector<float> ranges = laser_scan_msg->ranges;
    std::vector<float> intensities = laser_scan_msg->intensities;
    LaserScanPtr laser_scan_ptr = std::make_shared<LaserScan>
                                    (laser_scan_msg->header.stamp.toSec(), ranges, intensities,
                                    laser_scan_msg->angle_min, laser_scan_msg->angle_max, laser_scan_msg->angle_increment,
                                    laser_scan_msg->time_increment, laser_scan_msg->range_min, laser_scan_msg->range_max);
    return laser_scan_ptr;
}

OdomPtr SensorFactory::CreateOdomPtr(const nav_msgs::OdometryConstPtr &odom_msg) {
    OdomPtr odom_ptr = std::make_shared<Odom>(odom_msg->header.stamp.toSec());
    odom_ptr->q_.w() = odom_msg->pose.pose.orientation.w;
    odom_ptr->q_.x() = odom_msg->pose.pose.orientation.x;
    odom_ptr->q_.y() = odom_msg->pose.pose.orientation.y;
    odom_ptr->q_.z() = odom_msg->pose.pose.orientation.z;

    odom_ptr->t_.x() = odom_msg->pose.pose.position.x;
    odom_ptr->t_.y() = odom_msg->pose.pose.position.y;
    odom_ptr->t_.z() = odom_msg->pose.pose.position.z;

    return odom_ptr;
}

void SensorFactory::OdomToRosMsg(const OdomPtr &odom, nav_msgs::Odometry& odom_ros,
                    const std::string& frame_id, const std::string& child_frame_id) {
    odom_ros.header.frame_id = frame_id;
    odom_ros.child_frame_id = child_frame_id;
    OdomToRosMsg(odom, odom_ros);
}
void SensorFactory::OdomToRosMsg(const OdomPtr &odom, nav_msgs::Odometry& odom_ros) {
    odom_ros.header.stamp = ros::Time(odom->timestamp_);

    odom_ros.pose.pose.orientation.w = odom->q_.w();
    odom_ros.pose.pose.orientation.x = odom->q_.x();
    odom_ros.pose.pose.orientation.y = odom->q_.y();
    odom_ros.pose.pose.orientation.z = odom->q_.z();

    odom_ros.pose.pose.position.x = odom->t_.x();
    odom_ros.pose.pose.position.y = odom->t_.y();
    odom_ros.pose.pose.position.z = odom->t_.z();
}

void SensorFactory::LaserScanToRosMsg(const LaserScanPtr &laser, sensor_msgs::LaserScan& laser_ros) {
    laser_ros.header.stamp = ros::Time(laser->timestamp_);
    laser_ros.ranges = laser->ranges_;
    laser_ros.intensities = laser->intensities_;
    laser_ros.angle_min = laser->angle_min_;
    laser_ros.angle_max = laser->angle_max_;
    laser_ros.angle_increment = laser->angle_increment_;
    laser_ros.time_increment = laser->time_increment_;
    laser_ros.range_min = laser->range_min_;
    laser_ros.range_max = laser->range_max_;
}

void SensorFactory::ImuToRosMsg(const ImuPtr &imu, sensor_msgs::Imu &imu_ros) {
    imu_ros.header.stamp = ros::Time(imu->timestamp_);
    imu_ros.orientation.w = imu->orientation_.w();
    imu_ros.orientation.x = imu->orientation_.x();
    imu_ros.orientation.y = imu->orientation_.y();
    imu_ros.orientation.z = imu->orientation_.z();

    imu_ros.linear_acceleration.x = imu->linear_acceleration_.x();
    imu_ros.linear_acceleration.y = imu->linear_acceleration_.y();
    imu_ros.linear_acceleration.z = imu->linear_acceleration_.z();

    imu_ros.angular_velocity.x = imu->angular_velocity_.x();
    imu_ros.angular_velocity.y = imu->angular_velocity_.y();
    imu_ros.angular_velocity.z = imu->angular_velocity_.z();
}


}
