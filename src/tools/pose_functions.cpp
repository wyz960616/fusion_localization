//@Description
//@Author:wyz
//@Time:2021/1/4 下午2:41

#include "tools/pose_functions.h"
#include <cmath>
namespace fusion_localization {
bool PoseFunctions::Homogeneous4dto3d(const Eigen::Matrix4d &source, Eigen::Matrix3d &target) {
    Eigen::Quaterniond q(source.block<3,3>(0,0));
    double half_theta = acos(q.coeffs()[3]);
    Eigen::Vector3d axis = q.coeffs().block<3,1>(0,0)/sin(half_theta);
    double distance = sqrt( axis[0]*axis[0] + axis[1]*axis[1] );
    if(distance > 0.05) {
        return false;
    }
    Eigen::Vector3d trans = source.block<3,1>(0,3);
    if(trans[2] > 0.05) {
        return false;
    }
    q.x() = 0;
    q.y() = 0;
    q.normalize();

    target.setIdentity();
    target.block<2,2>(0,0) = q.toRotationMatrix().block<2,2>(0,0);
    target.block<2,1>(0,2) = trans.block<2,1>(0,0);

    return true;
}



}
