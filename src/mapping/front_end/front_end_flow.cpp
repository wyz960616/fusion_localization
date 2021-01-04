//@Description
//@Author:wyz
//@Time:2020/12/28 下午3:47                            

#include "mapping/front_end/front_end_flow.h"

namespace fusion_localization {

FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, const std::string &config_path) : nh_(nh){
    YAML::Node config_node = YAML::LoadFile(config_path);
    std::string imu_sub_topic = config_node["imu"]["sub_topic"].as<std::string>();
    int imu_sub_buf_size = config_node["imu"]["buf_size"].as<int>();
    imu_subscriber_ = std::make_shared<ImuSubscriber>(nh_, imu_sub_topic, imu_sub_buf_size);

    std::string odom_sub_topic = config_node["odom"]["sub_topic"].as<std::string>();
    int odom_sub_buf_size = config_node["odom"]["buf_size"].as<int>();
    odom_subscriber_ = std::make_shared<OdomSubscriber>(nh_, odom_sub_topic, odom_sub_buf_size);

    std::string laser_sub_topic = config_node["laser"]["sub_topic"].as<std::string>();
    int laser_sub_buf_size = config_node["laser"]["sub_buf_size"].as<int>();
    laser_subscriber_ = std::make_shared<LaserSubscriber>(nh_, laser_sub_topic, laser_sub_buf_size);

    std::string odom_pub_topic = config_node["odom"]["pub_topic"].as<std::string>();
    std::string odom_frame_id = config_node["odom"]["frame_id"].as<std::string>();
    std::string odom_child_frame_id = config_node["odom"]["child_frame_id"].as<std::string>();
    int odom_pub_buf_size = config_node["odom"]["pub_buf_size"].as<int>();
    laser_odom_publihser_ = std::make_shared<OdomPublisher>(nh_, odom_pub_topic, odom_pub_buf_size,
                                                      odom_frame_id, odom_child_frame_id);

    front_end_ptr_ = std::make_shared<FrontEnd>(config_node["front_end"]);
}


bool FrontEndFlow::ReadData() {
    laser_subscriber_->ParseData();
    odom_subscriber_->ParseData();
    imu_subscriber_->ParseData();
    double laser_scan_timestamp = laser_subscriber_->parsed_data_.front()->timestamp_;
    bool odom_synced = odom_subscriber_->SyncedData(laser_scan_timestamp);
    bool imu_synced = imu_subscriber_->SyncedData(laser_scan_timestamp);
    if(!odom_synced || !imu_synced) {
        laser_subscriber_->parsed_data_.pop_front();
        return false;
    }
    return true;
}

bool FrontEndFlow::HasData() {
    if(laser_subscriber_->parsed_data_.empty() || odom_subscriber_->parsed_data_.empty()
        || imu_subscriber_->parsed_data_.empty()){
        return false;
    }
}

bool FrontEndFlow::ValidData() {
    laser_scan_undistorted_ = CAST_TO_SCAN(laser_subscriber_->parsed_data_.front());
    odom_synced_ = CAST_TO_ODOM(odom_subscriber_->parsed_data_.front());
    imu_synced_ = CAST_TO_IMU(imu_subscriber_->parsed_data_.front());
    double laser_scan_timestamp = laser_scan_undistorted_->timestamp_;
    double odom_timestamp = odom_synced_->timestamp_;
    double imu_timestamp = imu_synced_->timestamp_;

    if(laser_scan_timestamp - odom_timestamp > 0.05 ) {
        odom_subscriber_->parsed_data_.pop_front();
        return false;
    }

    if(laser_scan_timestamp - odom_timestamp < -0.05) {
        laser_subscriber_->parsed_data_.pop_front();
        return false;
    }

    if(laser_scan_timestamp - imu_timestamp > 0.05) {
        imu_subscriber_->parsed_data_.pop_front();
        return false;
    }

    if(laser_scan_timestamp - imu_timestamp < -0.05) {
        laser_subscriber_->parsed_data_.pop_front();
        return false;
    }

    laser_subscriber_->parsed_data_.pop_front();
    odom_subscriber_->parsed_data_.pop_front();
    imu_subscriber_->parsed_data_.pop_front();
    return true;
}

bool FrontEndFlow::Run() {
    if(!ReadData()) {
        return false;
    }
    while(HasData()) {
        if(ValidData()) {
            if(front_end_ptr_->Match(laser_scan_undistorted_, odom_synced_, imu_synced_, precise_estimated_)) {
                laser_odom_->timestamp_ = laser_scan_undistorted_->timestamp_;
                laser_odom_->q_ = Eigen::Quaterniond(precise_estimated_.block<3,3>(0,0));
                laser_odom_->t_ = Eigen::Vector3d(precise_estimated_.block<3,1>(0,3));
            }else {
                //打滑处理？
                laser_odom_->timestamp_ = laser_scan_undistorted_->timestamp_;
                laser_odom_->q_ = Eigen::Quaterniond(precise_estimated_.block<3,3>(0,0));
                laser_odom_->t_ = Eigen::Vector3d(precise_estimated_.block<3,1>(0,3));
            }
            Publish();
        }
    }
}

void FrontEndFlow::Publish() {
    laser_odom_publihser_->Publish(laser_odom_);
}

}
