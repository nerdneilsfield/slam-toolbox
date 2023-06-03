#include "slam_toolbox/points.hpp"

SimplePointCloud::SimplePointCloud() {
    points = std::vector<SimplePoint>();
}

SimplePointCloud::SimplePointCloud(const std::vector<SimplePoint>& points) {
    this->points = points;
}

SimplePointCloud::SimplePointCloud(const PointCloudPtr cloud) {
    points = std::vector<SimplePoint>();
    for (auto point : *cloud) {
        points.emplace_back(SimplePoint{point.x, point.y, point.z, point.intensity});
    }
}

SimplePointCloud::SimplePointCloud(std::vector<SimplePoint>&& points) {
    this->points = std::move(points);
}

std::size_t SimplePointCloud::GetSize() const {
    return points.size();
}

void SimplePointCloud::Clear() {
    points.clear();
}

void SimplePointCloud::AddPoint(const SimplePoint& point) {
    points.emplace_back(point);
}


void SimplePointCloud::AddPoint(SimplePoint&& point) {
    points.emplace_back(std::move(point));
}

void SimplePointCloud::AddPoint(const PointType& point) {
    points.emplace_back(SimplePoint{point.x, point.y, point.z, point.intensity});
}

void SimplePointCloud::AddPoint(PointType&& point) {
    points.emplace_back(SimplePoint{point.x, point.y, point.z, point.intensity});
}

void SimplePointCloud::AddPoints(const SimplePointCloud& other) {
    for (auto point : other.points) {
        AddPoint(point);
    }
}

void SimplePointCloud::AddPoints(const std::vector<SimplePoint>& points) {
    for (auto point : points) {
        AddPoint(point);
    }
}

void SimplePointCloud::AddPoints(std::vector<SimplePoint>&& points) {
    for (auto point : points) {
        AddPoint(std::move(point));
    }
}



void SimplePointCloud::AddPoints(const PointCloudPtr cloud) {
    for (auto point : *cloud) {
        AddPoint(point);
    }
}

void SimplePointCloud::AddPoints(PointCloudPtr&& cloud) {
    for (auto point : *cloud) {
        AddPoint(std::move(point));
    }
}

void SimplePointCloud::AddPoints(const std::vector<PointType>& points) {
    for (auto point : points) {
        AddPoint(point);
    }
}

void SimplePointCloud::AddPoints(std::vector<PointType>&& points) {
    for (auto point : points) {
        AddPoint(std::move(point));
    }
}


PointCloud SimplePointCloud::ToPclPointCloud() {
    PointCloud cloud;
    for (auto point : points) {
        PointType p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.intensity = point.intensity;
        cloud.emplace_back(p);
    }
    return cloud;
}