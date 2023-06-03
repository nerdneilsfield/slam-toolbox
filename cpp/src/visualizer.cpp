#include "slam_toolbox/visualizer.hpp"

#include <cstdio>
#include <cstdlib>
#include <pcl/impl/point_types.hpp>
#include <spdlog/spdlog.h>
#include <thread>

Visualizer::Visualizer(const std::string &window_name,
                       const std::string &color) {
  spdlog::debug("Initializing visualizer");
  try {
    viewer_ = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer(window_name));
  } catch (const std::exception &e) {
    spdlog::error("Could not initialize visualizer: {}", e.what());
    throw e;
  }
  viewer_->setBackgroundColor(0, 0, 0);
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointXYZRGB p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  p.r = 255;
  p.g = 255;
  p.b = 255;
  cloud->push_back(p);

  viewer_->updatePointCloud(cloud, window_name);

  uint8_t r = 0, g = 0, b = 0;
  if (color == "r") {
    r = 255;
  } else if (color == "g") {
    r = 0;
    g = 255;
  } else if (color == "b") {
    g = 0;
    b = 255;
  } else if (color == "y") {
    r = 255;
    g = 255;
  } else if (color == "c") {
    g = 255;
    b = 255;
  } else if (color == "m") {
    r = 255;
    b = 255;
  } else if (color == "w") {
    r = 255;
    g = 255;
    b = 255;
  } else if (color == "k") {
    r = 0;
    g = 0;
    b = 0;
  }
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, window_name);
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, r / 255.0, g / 255.0, b / 255.0,
      window_name);

  char id_text[128];
  sprintf(id_text, "Frame: %d", -1);
  viewer_->addText(id_text, 20, 20, 0, 0, 0, "id");
  viewer_->initCameraParameters();
  viewer_->setCameraPosition(-4, 0, 2.5, 0, 0, 2.5);
}

void Visualizer::UpdatePointCloud(const SimplePointCloud &point_cloud,
                                  const std::string &name,
                                  const std::string &color,
                                  const uint32_t &id) {
  PointCloud cloud =
      const_cast<SimplePointCloud &>(point_cloud).ToPclPointCloud();
  this->UpdatePointCloud(cloud.makeShared(), name, color, id);
}

void Visualizer::UpdatePointCloud(const PointCloudPtr &point_cloud,
                                  const std::string &name,
                                  const std::string &color,
                                  const uint32_t &id) {

  spdlog::debug("Updating point cloud: {} with points {} and id {}", name,
                point_cloud->points.size(), id);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  for (auto &point : *point_cloud) {
    pcl::PointXYZRGB p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    p.r = 255;
    p.g = 255;
    p.b = 255;
    cloud->push_back(p);
  }

  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  if (color == "r") {
    r = 255;
  } else if (color == "g") {
    r = 0;
    g = 255;
  } else if (color == "b") {
    g = 0;
    b = 255;
  } else if (color == "y") {
    r = 255;
    g = 255;
  } else if (color == "c") {
    g = 255;
    b = 255;
  } else if (color == "m") {
    r = 255;
    b = 255;
  } else if (color == "w") {
    r = 255;
    g = 255;
    b = 255;
  } else if (color == "k") {
    r = 0;
    g = 0;
    b = 0;
  }

  if (!viewer_->updatePointCloud(cloud, name)) {
    spdlog::warn("Could not update point cloud: {}", name);
    viewer_->addPointCloud(cloud, name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, r / 255.0, g / 255.0,
        b / 255.0, name);
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(-4, 0, 2.5, 0, 0, 2.5);
  }
  char id_text[128];
  sprintf(id_text, "Frame: %d", id);
  viewer_->updateText(id_text, 20, 20, 0, 0, 0, "id");
  viewer_->spinOnce(250);
}

bool Visualizer::WasStopped() { return viewer_->wasStopped(); }

void Visualizer::SpinOnce(int32_t time) { viewer_->spinOnce(time); }
