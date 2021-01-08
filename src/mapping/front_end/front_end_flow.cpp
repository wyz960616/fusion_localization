//@Description
//@Author:wyz
//@Time:2020/12/28 下午3:47                            

#include "mapping/front_end/front_end_flow.h"

namespace fusion_localization {
/*!
 * @breif 由配置文件的参数，完成FrontEndFlow中3个subscriber,1个publisher,以及front_end的初始化
 * @param nh
 * @param config_path config文件的具体路径
 */
FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, const std::string &config_path) {
    nh_ = nh;
    std::cout << config_path << std::endl;
    std::cout << "---------------初始化front_end_flow节点-------------------" << std::endl;
    YAML::Node config_node = YAML::LoadFile(config_path);
    std::string imu_sub_topic = config_node["imu"]["sub_topic"].as<std::string>();
    int imu_sub_buf_size = config_node["imu"]["sub_buf_size"].as<int>();
    imu_subscriber_ = std::make_shared<ImuSubscriber>(nh_, imu_sub_topic, imu_sub_buf_size);
    std::cout << "imu_topic:" << imu_sub_topic << std::endl;

    std::string odom_sub_topic = config_node["odom"]["sub_topic"].as<std::string>();
    int odom_sub_buf_size = config_node["odom"]["sub_buf_size"].as<int>();
    odom_subscriber_ = std::make_shared<OdomSubscriber>(nh_, odom_sub_topic, odom_sub_buf_size);
    std::cout << "odom_sub_topic:" << odom_sub_topic << std::endl;

    std::string laser_sub_topic = config_node["laser"]["sub_topic"].as<std::string>();
    int laser_sub_buf_size = config_node["laser"]["sub_buf_size"].as<int>();
    laser_subscriber_ = std::make_shared<LaserSubscriber>(nh_, laser_sub_topic, laser_sub_buf_size);
    std::cout << "laser_topic:" << laser_sub_topic << std::endl;

    std::string odom_pub_topic = config_node["odom"]["pub_topic"].as<std::string>();
    std::string odom_frame_id = config_node["odom"]["frame_id"].as<std::string>();
    std::string odom_child_frame_id = config_node["odom"]["child_frame_id"].as<std::string>();
    int odom_pub_buf_size = config_node["odom"]["pub_buf_size"].as<int>();
    laser_odom_publihser_ = std::make_shared<OdomPublisher>(nh_, odom_pub_topic, odom_pub_buf_size,
                                                      odom_frame_id, odom_child_frame_id);
    YAML::Node node2 = config_node["front_end"];
    front_end_ptr_ = std::make_shared<FrontEnd>(node2);
}


/*!
 * @breif 带锁读取数据，并完成数据的时间戳对齐
 * front_end读取的是data_pretreat中发来预处理过的数据，3个传感器时间戳应完全一致
 * @return 对齐失败返回false
 */
bool FrontEndFlow::ReadData() {
    laser_subscriber_->ParseData();
    odom_subscriber_->ParseData();
    imu_subscriber_->ParseData();
    if(laser_subscriber_->parsed_data_.empty() || odom_subscriber_->parsed_data_.empty()
       || imu_subscriber_->parsed_data_.empty()){
        return false;
    }
    double laser_scan_timestamp = laser_subscriber_->parsed_data_.front()->timestamp_;
    laser_subscriber_->synced_data_.push_back(laser_subscriber_->parsed_data_.front());
    laser_subscriber_->parsed_data_.pop_front();

    bool odom_synced = odom_subscriber_->SyncedData(laser_scan_timestamp);
    bool imu_synced = imu_subscriber_->SyncedData(laser_scan_timestamp);
    if(!odom_synced || !imu_synced) {
        laser_subscriber_->synced_data_.pop_front();
        return false;
    }
    return true;
}

bool FrontEndFlow::HasData() {
    if(laser_subscriber_->synced_data_.empty() || odom_subscriber_->synced_data_.empty()
        || imu_subscriber_->synced_data_.empty()){
        return false;
    }
    return true;
}

bool FrontEndFlow::ValidData() {
    laser_scan_undistorted_ = CAST_TO_SCAN(laser_subscriber_->synced_data_.front());
    odom_synced_ = CAST_TO_ODOM(odom_subscriber_->synced_data_.front());
    imu_synced_ = CAST_TO_IMU(imu_subscriber_->synced_data_.front());
    double laser_scan_timestamp = laser_scan_undistorted_->timestamp_;
    double odom_timestamp = odom_synced_->timestamp_;
    double imu_timestamp = imu_synced_->timestamp_;

    if(laser_scan_timestamp - odom_timestamp > 0.05 ) {
        odom_subscriber_->synced_data_.pop_front();
        return false;
    }

    if(laser_scan_timestamp - odom_timestamp < -0.05) {
        laser_subscriber_->synced_data_.pop_front();
        return false;
    }

    if(laser_scan_timestamp - imu_timestamp > 0.05) {
        imu_subscriber_->synced_data_.pop_front();
        return false;
    }

    if(laser_scan_timestamp - imu_timestamp < -0.05) {
        laser_subscriber_->synced_data_.pop_front();
        return false;
    }

    laser_subscriber_->synced_data_.pop_front();
    odom_subscriber_->synced_data_.pop_front();
    imu_subscriber_->synced_data_.pop_front();
    return true;
}

/*!
 * @breif 由外部调用的运行态函数，读取数据后，完成前端校准，并发布到ROS相应的topic中
 * @return
 */
bool FrontEndFlow::Run() {
    if(!ReadData()) {
        return false;
    }
    while(HasData()) {
        if(ValidData()) {
            // TODO
            // front_end利用imu中的线速度，假定机器人短时间内匀速运动，预估机器人下一帧的位置，以完成一下情况的处理:
            // 匹配失败的可能
            //1. 里程计打滑，给出的odom作为初值不准确 ------- 直接使用预估值，再进行一次Match
            //2. 环境发生突变，点云重合度过低 ------- 直接取odom和预估值的中值作为输出
            if(front_end_ptr_->Match(laser_scan_undistorted_, odom_synced_, imu_synced_, precise_estimated_)) {
                laser_odom_->timestamp_ = laser_scan_undistorted_->timestamp_;
                laser_odom_->q_ = Eigen::Quaterniond(precise_estimated_.block<3,3>(0,0));
                laser_odom_->t_ = Eigen::Vector3d(precise_estimated_.block<3,1>(0,3));
                Publish();
            }else {

            }
            //Publish();
        }
    }
    return true;
}

void FrontEndFlow::Publish() {
    laser_odom_publihser_->Publish(laser_odom_);
}

}
