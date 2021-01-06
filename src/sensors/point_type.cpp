//@Description
//@Author:wyz
//@Time:2021/1/6 下午3:16

#include "sensors/point_type.h"
#include <cmath>
namespace fusion_localization {
/*!
 * @breif 在Mat上绘制一个点云数据
 * @param cloud
 * @param color
 * @param out
 */
void PointTypes::DrawPointCloud(const PointTypes::CLOUD &cloud, cv::Scalar& color, cv::Mat& out) {
    float min_x = 1e15, min_y = 1e15, max_x = -1e15, max_y = -1e15;
    FindBoundary(cloud, min_x, min_y, max_x, max_y);
    int col_size = static_cast<int>(std::floor(max_x - min_x) + 5);
    int row_size = static_cast<int>(std::floor(max_y - min_y) + 5);
    out = cv::Mat::zeros(cv::Size(col_size, row_size), CV_8UC3);
    out.setTo(255);

    for(int i = 0 ; i < cloud.size() ; ++i) {
        int col = static_cast<int>(std::ceil(cloud[i].x - min_x));
        int row = static_cast<int>(std::ceil(cloud[i].y - min_y));
        cv::Point point(col, row);
        cv::circle(out, point, 1, color, -1);
    }

}

/*!
 * @breif 在Mat上绘制两个二维点云数据
 * @param cloud1
 * @param cloud2
 * @param color1
 * @param color2
 * @param out
 */
void PointTypes::DrawPointCloud(const PointTypes::CLOUD &cloud1, const PointTypes::CLOUD &cloud2,
                           const cv::Scalar &color1, const cv::Scalar &color2, cv::Mat& out) {
    float min_x = 1e15, min_y = 1e15, max_x = -1e15, max_y = -1e15;
    FindBoundary(cloud1, min_x, min_y, max_x, max_y);
    FindBoundary(cloud2, min_x, min_y, max_x, max_y);
    int col_size = static_cast<int>(std::floor(max_x - min_x) + 5);
    int row_size = static_cast<int>(std::floor(max_y - min_y) + 5);
    out = cv::Mat::zeros(cv::Size(col_size, row_size), CV_8UC3);
    out.setTo(255);

    for(int i = 0 ; i < cloud1.size() ; ++i) {
        int col = static_cast<int>(std::ceil(cloud1[i].x - min_x));
        int row = static_cast<int>(std::ceil(cloud1[i].y - min_y));
        cv::Point point(row, col);
        cv::circle(out, point, 2, color1, -1);
    }

    for(int i = 0 ; i < cloud2.size() ; ++i) {
        int col = static_cast<int>(std::ceil(cloud2[i].x - min_x));
        int row = static_cast<int>(std::ceil(cloud2[i].y - min_y));
        cv::Point point(col, row);
        cv::circle(out, point, 1, color2, -1);
    }
}

/*!
 * 确定给定点云的4个边界值
 * @param cloud
 * @param min_x
 * @param min_y
 * @param max_x
 * @param max_y
 */
void PointTypes::FindBoundary(const PointTypes::CLOUD &cloud, float &min_x, float &min_y, float &max_x, float &max_y) {
    for(int i = 0 ; i < cloud.size() ; ++i) {
        if(cloud[i].x < min_x) {
            min_x = cloud[i].x;
        }
        if(cloud[i].y < min_y) {
            min_y = cloud[i].y;
        }
        if(cloud[i].x > max_x) {
            max_x = cloud[i].x;
        }
        if(cloud[i].y > max_y) {
            max_y = cloud[i].y;
        }
    }
}



}