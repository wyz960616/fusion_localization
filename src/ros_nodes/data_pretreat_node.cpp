//@Description
//@Author:wyz
//@Time:2020/12/18 下午1:41                            

#include "data_pretreat/data_pretreat.h"
#include "global_definition.h"
#include <ros/ros.h>

using namespace fusion_localization;
int main(int argc, char** argv) {
    ros::init(argc, argv, "data_pretreat");
    ros::NodeHandle nh;
    std::string config_path = WORK_SPACE_PATH + "/config/data_pretreat.yaml";
    std::shared_ptr<DataPretreat> data_pretreat_ptr = std::make_shared<DataPretreat>(nh, config_path);

    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        data_pretreat_ptr->Run();
        rate.sleep();
    }

    return 0;
}