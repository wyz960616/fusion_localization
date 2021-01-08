//@Description
//@Author:wyz
//@Time:2021/1/6 下午1:53

#ifndef LIDAR_LOCALIZATION_POINT_DATA_H
#define LIDAR_LOCALIZATION_POINT_DATA_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define CLOUD_COLOR_RED cv::Scalar(31,23,176)
#define CLOUD_COLOR_BLACK cv::Scalar(88,87,86)
#define CLOUD_COLOR_GRAY cv::Scalar(192,192,192)
#define CLOUD_COLOR_YELLOW cv::Scalar(205,235,255)
#define CLOUD_COLOR_GREEN cv::Scalar(84,46,8)

namespace fusion_localization {
class PointTypes {
public:
    typedef pcl::PointXYZI POINT_DATA;
    typedef pcl::PointCloud<POINT_DATA> CLOUD;
    typedef CLOUD::Ptr CLOUD_PTR;
    static void DrawPointCloud(const PointTypes::CLOUD &cloud, cv::Mat& out, cv::Scalar color = CLOUD_COLOR_RED);
    static void DrawPointCloud(const PointTypes::CLOUD &cloud1,const PointTypes::CLOUD &cloud2,
                               cv::Mat& out, cv::Scalar color1 = CLOUD_COLOR_RED, cv::Scalar color2 = CLOUD_COLOR_GREEN);
    static void DrawCompare(const PointTypes::CLOUD &back_cloud, const PointTypes::CLOUD &origin_cloud, const PointTypes::CLOUD &aligned_cloud,
                            cv::Mat& out, cv::Scalar color1 = CLOUD_COLOR_RED, cv::Scalar color2 = CLOUD_COLOR_GREEN);

protected:
    static void FindBoundary(const PointTypes::CLOUD &cloud, float &min_x, float &min_y, float &max_x, float &max_y);
};

}


#endif //LIDAR_LOCALIZATION_POINT_DATA_H
