//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:12                            

#include "publishers/odom_publihser.h"

namespace fusion_localization {

OdomPublihser::OdomPublihser(const ros::NodeHandle &nh, const std::string &odom_topic, int buf_size,
                             const std::string &frame_id, const std::string &child_frame_id) {
    nh_ = nh;
    publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, buf_size);
    odom_ros_msg_.header.frame_id = frame_id;
    odom_ros_msg_.child_frame_id = child_frame_id;
}

void OdomPublihser::Publish(const OdomPtr &odom_ptr) {
    Odom::ToRosMsg(odom_ptr, odom_ros_msg_);
    publisher_.publish(odom_ptr);
}

}
