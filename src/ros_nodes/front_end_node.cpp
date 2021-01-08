//@Description
//@Author:wyz
//@Time:2021/1/6 上午11:49
#include<ros/ros.h>
#include"mapping/front_end/front_end_flow.h"
#include "global_definition.h"
using namespace fusion_localization;

int main(int argc, char **argv) {
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::string config_path = WORK_SPACE_PATH + "/config/front_end_flow.yaml";
    std::shared_ptr<FrontEndFlow> front_end_flow = std::make_shared<FrontEndFlow>(nh, config_path);

    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        front_end_flow->Run();
        rate.sleep();
    }

    return 0;
}