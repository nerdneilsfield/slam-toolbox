#ifndef __SLAM_TOOLBOX_POINTS_H__
#define __SLAM_TOOLBOX_POINTS_H__

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;

/**
 * @brief A simple point struct
 */
struct SimplePoint {
    float x;
    float y;
    float z;
    float intensity;
}; // struct SimplePoint

/**
 * @brief A simple point cloud struct
 */
struct SimplePointCloud {
    std::vector<SimplePoint> points;

    SimplePointCloud();

    SimplePointCloud(const std::vector<SimplePoint>& points);

    SimplePointCloud(const PointCloudPtr cloud);

    SimplePointCloud(std::vector<SimplePoint>&& points);

    [[nodiscard]] std::size_t GetSize() const;

    void Clear();

    void AddPoint(const SimplePoint& point);

    void AddPoint(SimplePoint&& point);

    void AddPoint(const PointType& point);

    void AddPoint(PointType&& point);

    void AddPoints(const SimplePointCloud& other);

    void AddPoints(SimplePointCloud&& other);

    void AddPoints(const PointCloudPtr cloud);

    void AddPoints(PointCloudPtr&& cloud);

    void AddPoints(const std::vector<SimplePoint>& points);

    void AddPoints(std::vector<SimplePoint>&& points);

    void AddPoints(const std::vector<PointType>& points);

    void AddPoints(std::vector<PointType>&& points);

    PointCloud ToPclPointCloud();
    
}; // struct SimplePointCloud



#endif // __SLAM_TOOLBOX_BINARY_H__