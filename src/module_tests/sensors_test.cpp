//@Description
//@Author:wyz
//@Time:2020/12/17 下午5:27                            

#include "subscribers/odom_subscriber.h"
#include "subscribers/laser_subscriber.h"
#include <ros/ros.h>
#include <memory>
#include <iostream>

using namespace std;
using namespace fusion_localization;

std::shared_ptr<OdomSubscriber> odom_subscriber;
std::shared_ptr<LaserSubscriber> laser_subscriber;
int iter = 0;
bool ReadData() {
    static std::deque<SensorPtr>& un_synced_laser_data = laser_subscriber->parsed_data_;
    static std::deque<SensorPtr>& un_synced_odom_data = odom_subscriber->parsed_data_;
    static std::deque<SensorPtr>& synced_laser_data = laser_subscriber->synced_data_;
    static std::deque<SensorPtr>& synced_odom_data = odom_subscriber->synced_data_;

    laser_subscriber->ParseData();
    odom_subscriber->ParseData();
    if(un_synced_laser_data.empty()) {
        return false;
    }

    double scan_time_stamp = un_synced_laser_data.front()->timestamp_;
    if(odom_subscriber->SyncedData(scan_time_stamp)) {
        cout << "SyncedData "<< iter++ << endl;
        auto synced_data = CAST_TO_ODOM(synced_odom_data.back());
        cout.setf(ios::fixed,ios::floatfield);//十进制计数法，不是科学计数法
        cout.precision(10);
        cout << "校准时间戳: " << scan_time_stamp << " 对齐时间戳: "<< synced_data->timestamp_ << endl
             << synced_data->Pose() << endl;
        un_synced_laser_data.pop_front();
    }
    return true;
}

bool ValidData() {
    return false;
}

int main(int argc, char**argv) {
    ros::init(argc, argv,"sensor_test");
    ros::NodeHandle nh;
    odom_subscriber = std::make_shared<OdomSubscriber>(nh, "odom", 10000);
    laser_subscriber = std::make_shared<LaserSubscriber>(nh, "scan", 1000);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        ReadData();
        rate.sleep();
    }

}