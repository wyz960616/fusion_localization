//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:17                            

#ifndef LIDAR_LOCALIZATION_ICP_REGISTRATION_H
#define LIDAR_LOCALIZATION_ICP_REGISTRATION_H

#include "models/scan_matches/scan_registration.h"
#include "models/scan_matches/local_map.h"
#include <yaml-cpp/yaml.h>

namespace fusion_localization {
class ICPRegistration : public ScanRegistration{
public:
    ICPRegistration(const YAML::Node& config_node);
    /*!
     * @breif 激光雷达帧与帧之间的匹配，调用此函数前需要设置好此次帧间匹配的laser_scan
     * @param precise_estimated 雷达匹配之后位姿估计的返回值
     * @return 若匹配失败返回false
     */
    bool ScanToScan(Eigen::Matrix4d &precise_estimated);
    /*!
    * @breif 激光雷达帧与图之间的匹配，调用此函数前需要设置好此次帧间匹配的laser_scan
    * @param precise_estimated 雷达匹配之后位姿估计的返回值
    * @return 若匹配失败返回false
    */
    bool ScanToMap(Eigen::Matrix4d &precise_estimated);
    /*!
     * @breif 根据配置文件中的参数选择 ScanToMap/ScanToScan
     * @param precise_estimated 雷达匹配之后位姿估计的返回值
     * @return 若匹配失败返回false
     */
    bool Match(Eigen::Matrix4d &precise_estimated) override;

private:
    int max_iteration_;
    float resolution_;
    bool scan2map_;
//ScanToMap额外需要一个local_map
private:
    std::shared_ptr<LocalMap> local_map_;
    //n帧scan形成一张local_map
    int scan_size_{};
};

}


#endif //LIDAR_LOCALIZATION_ICP_REGISTRATION_H
