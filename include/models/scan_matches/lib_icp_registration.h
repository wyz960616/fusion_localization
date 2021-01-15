//@Description
//@Author:wyz
//@Time:2021/1/14 上午11:38

#ifndef LIDAR_LOCALIZATION_LIB_ICP_REGISTRATION_H
#define LIDAR_LOCALIZATION_LIB_ICP_REGISTRATION_H
#include "libicp/include/icpPointToPoint.h"
#include "libicp/include/icpPointToPlane.h"
#include "models/scan_matches/scan_registration.h"
#include <yaml-cpp/yaml.h>
namespace fusion_localization {
class LibIcpRegistration :public ScanRegistration{
public:
    LibIcpRegistration(YAML::Node config_node);
    bool ScanToScan(Eigen::Matrix4d &precise_estimated);
    bool ScanToMap(Eigen::Matrix4d &precise_estimated);
    bool Match(Eigen::Matrix4d &precise_estimated) override;

    typedef enum{Point2Point, Point2Plane} LIBICP_TYPE;
protected:
    int TransToDoubleVector(double* M, const LaserScanPtr &laser_ptr);
    int TransToWord(double* M, int size, const Eigen::Matrix2d &R, const Eigen::Vector2d &t);
private:
    bool scan2map_ = false;
    int dim_ = 2;
    double outlier_ = -1;
    LIBICP_TYPE type_;
};
}


#endif //LIDAR_LOCALIZATION_LIB_ICP_REGISTRATION_H
