//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:12                            

#ifndef LIDAR_LOCALIZATION_ODOM_PUBLIHSER_H
#define LIDAR_LOCALIZATION_ODOM_PUBLIHSER_H
#include "sensors/odom.h"
#include <ros/ros.h>
namespace fusion_localization {
class OdomPublihser {
public:
    OdomPublihser(const ros::NodeHandle &nh, const std::string &odom_topic, int buf_size,
                  const std::string &frame_id, const std::string &child_frame_id);
    void Publish(const OdomPtr &odom_ptr);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odom_ros_msg_;
};

}


#endif //LIDAR_LOCALIZATION_ODOM_PUBLIHSER_H
