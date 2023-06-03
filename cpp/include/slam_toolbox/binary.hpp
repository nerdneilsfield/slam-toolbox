#ifndef __SLAM_TOOLBOX_BINARY_H__
#define __SLAM_TOOLBOX_BINARY_H__

#include "points.hpp"
#include "slam_toolbox/points.hpp"

#include <cstdint>
#include <string>


/**
 * @brief A class for reading and writing binary files
 * 
 */
class BinaryIO {

public:
  BinaryIO();

  BinaryIO(const std::string &file_name);

  void SetFileName(const std::string &file_name);

  void LoadFromFile();

  void DumpToFile();

  void SetPointCloud(const SimplePointCloud &point_cloud);

  void SetPointCloud(SimplePointCloud &&point_cloud);

  void SetPointCloud(const PointCloudPtr& cloud);

  void SetPointCloud(PointCloudPtr &&cloud);

  [[nodiscard]] SimplePointCloud GetPointCloud();

  [[nodiscard]] PointCloud GetPclPointCloud();

private:
  std::string file_name_;
  SimplePointCloud point_cloud_;
}; // class BinaryIO

#endif // __SLAM_TOOLBOX_BINARY_H__