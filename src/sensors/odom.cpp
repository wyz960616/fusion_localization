//@Description
//@Author:wyz
//@Time:2020/12/17 下午7:58                            

#include "sensors/odom.h"

namespace fusion_localization {

Eigen::Matrix4d Odom::Pose() {
    Eigen::Matrix4d pose;
    pose.setIdentity();
    pose.block<3,3>(0, 0) = q_.toRotationMatrix();
    pose.block<3,1>(0, 3) = t_;
    return pose;
}

//SyncData中调用该重载函数，完成时间戳对齐操作
bool Odom::SetMedianValue(const SensorPtr &front_ptr, const SensorPtr &back_ptr, double synced_timestamp,
                          SensorPtr &median_ptr) {
    double front_scale = 0.5, back_scale = 0.5;
    MedianScale(front_ptr->timestamp_, back_ptr->timestamp_, synced_timestamp,
                front_scale, back_scale);
    median_ptr = std::make_shared<Odom>(synced_timestamp);
    OdomPtr odom_front_ptr = CAST_TO_ODOM(front_ptr);
    OdomPtr odom_back_ptr = CAST_TO_ODOM(back_ptr);
    OdomPtr odom_median_ptr = CAST_TO_ODOM(median_ptr);

    odom_median_ptr->q_.w() = front_scale * odom_front_ptr->q_.w() + back_scale * odom_back_ptr->q_.w();
    odom_median_ptr->q_.x() = front_scale * odom_front_ptr->q_.x() + back_scale * odom_back_ptr->q_.x();
    odom_median_ptr->q_.y() = front_scale * odom_front_ptr->q_.y() + back_scale * odom_back_ptr->q_.y();
    odom_median_ptr->q_.z() = front_scale * odom_front_ptr->q_.z() + back_scale * odom_back_ptr->q_.z();

    odom_median_ptr->t_ = front_scale * odom_front_ptr->t_ + back_scale * odom_back_ptr->t_;

    return true;
}

std::string Odom::Name() {
    return "odom-sensor";
}

}
