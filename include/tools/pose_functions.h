//@Description
//@Author:wyz
//@Time:2021/1/4 下午2:41

#ifndef LIDAR_LOCALIZATION_POSE_FUNCTIONS_H
#define LIDAR_LOCALIZATION_POSE_FUNCTIONS_H
#include <Eigen/Core>
#include <Eigen/Dense>

namespace fusion_localization {
class PoseFunctions {
public:
    /*!
    * @breif 4维齐次坐标到3维的转换
    * @param source 原始的四维齐次坐标
    * @param target 转换后的三维齐次坐标（绕着轴 (0,0,1)旋转）
    * @return 倘若原始的四维齐次坐标存在较大的非yaw角转动,或存在较大的z轴平移则 返回false
    */
    static bool Homogeneous4dto3d(const Eigen::Matrix4d &source, Eigen::Matrix3d &target);
};
}


#endif //LIDAR_LOCALIZATION_POSE_FUNCTIONS_H
