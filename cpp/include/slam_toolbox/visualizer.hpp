#ifndef __SLAM_TOOLBOX_VISUALIZER_H__
#define __SLAM_TOOLBOX_VISUALIZER_H__

#include "slam_toolbox/points.hpp"
#include "slam_toolbox/utils.hpp"

#include <pcl/visualization/pcl_visualizer.h>

class Visualizer {
public:
  Visualizer(const std::string& window_name="SLAM Toolbox Visulizer", const std::string& color="r");

  void UpdatePointCloud(const SimplePointCloud &point_cloud,
                        const std::string &name, const std::string &color = "k",
                        const uint32_t &id = 0);

  void UpdatePointCloud(const PointCloudPtr &point_cloud,
                        const std::string &name, const std::string &color = "k",
                        const uint32_t &id = 0);

  bool WasStopped();

  void SpinOnce(int32_t time = 100);

private:
  pcl::visualization::PCLVisualizer::Ptr viewer_;
}; // class Visualizer

#endif // __SLAM_TOOLBOX_VISUALIZER_H__