//@Description
//@Author:wyz
//@Time:2021/1/6 下午1:53

#ifndef LIDAR_LOCALIZATION_POINT_DATA_H
#define LIDAR_LOCALIZATION_POINT_DATA_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define CLOUD_COLOR_RED
namespace fusion_localization {
class PointTypes {
public:
    typedef pcl::PointXYZI POINT_DATA;
    typedef pcl::PointCloud<POINT_DATA> CLOUD;
    typedef CLOUD::Ptr CLOUD_PTR;
    static void DrawPointCloud(const PointTypes::CLOUD &cloud, cv::Scalar& color, cv::Mat& out);
    static void DrawPointCloud(const PointTypes::CLOUD &cloud1,const PointTypes::CLOUD &cloud2,
                               const cv::Scalar &color1, const cv::Scalar &color2, cv::Mat& out);
    static void DrawCompare(const PointTypes::CLOUD &back_cloud, const PointTypes::CLOUD &origin_cloud, const PointTypes::CLOUD &aligned_cloud,
                            cv::Scalar color1 = (255,255,0),
                            );

protected:
    static void FindBoundary(const PointTypes::CLOUD &cloud, float &min_x, float &min_y, float &max_x, float &max_y);
};

}


#endif //LIDAR_LOCALIZATION_POINT_DATA_H
