//@Description
//@Author:wyz
//@Time:2020/12/17 下午8:19                            

#include "sensors/imu.h"

namespace fusion_localization {
Eigen::Matrix3d Imu::Rotation(){
    return orientation_.toRotationMatrix();
}
Imu::Imu(const sensor_msgs::ImuConstPtr& imu_msg) : Sensor(imu_msg->header.stamp.toSec()) {
    orientation_.w() = imu_msg->orientation.w;
    orientation_.x() = imu_msg->orientation.x;
    orientation_.y() = imu_msg->orientation.y;
    orientation_.z() = imu_msg->orientation.z;

    linear_acceleration_.x() = imu_msg->linear_acceleration.x;
    linear_acceleration_.y() = imu_msg->linear_acceleration.y;
    linear_acceleration_.z() = imu_msg->linear_acceleration.z;

    angular_velocity_.x() = imu_msg->angular_velocity.x;
    angular_velocity_.y() = imu_msg->angular_velocity.y;
    angular_velocity_.z() = imu_msg->angular_velocity.z;
}

bool Imu::SetMedianValue(const SensorPtr& front_ptr, const SensorPtr& back_ptr, double synced_timestamp,
                    SensorPtr& median_ptr) {
    double front_scale = 0.5, back_scale = 0.5;
    MedianScale(front_ptr->timestamp_, back_ptr->timestamp_, synced_timestamp,
                front_scale, back_scale);
    median_ptr = std::make_shared<Imu>(synced_timestamp);
    ImuPtr imu_front_ptr = CAST_TO_IMU(front_ptr);
    ImuPtr imu_back_ptr = CAST_TO_IMU(back_ptr);
    ImuPtr imu_median_ptr = CAST_TO_IMU(median_ptr);

    imu_median_ptr->linear_acceleration_ = front_scale * imu_front_ptr->linear_acceleration_ +
                                           back_scale * imu_back_ptr->linear_acceleration_;
    imu_median_ptr->angular_velocity_ = front_scale * imu_front_ptr->angular_velocity_ +
                                        back_scale * imu_back_ptr->angular_velocity_;

    imu_median_ptr->orientation_.w() = front_scale * imu_front_ptr->orientation_.w() +
                                       back_scale * imu_front_ptr->orientation_.w();
    imu_median_ptr->orientation_.x() = front_scale * imu_front_ptr->orientation_.x() +
                                       back_scale * imu_front_ptr->orientation_.x();
    imu_median_ptr->orientation_.y() = front_scale * imu_front_ptr->orientation_.y() +
                                       back_scale * imu_front_ptr->orientation_.y();
    imu_median_ptr->orientation_.z() = front_scale * imu_front_ptr->orientation_.z() +
                                       back_scale * imu_front_ptr->orientation_.z();

    return true;
}

std::string Imu::Name() {
    return "imu-sensor";
}
}
