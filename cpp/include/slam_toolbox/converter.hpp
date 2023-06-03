#ifndef __SLAM_TOOLBOX_CONVERTER_H__
#define __SLAM_TOOLBOX_CONVERTER_H__

#include "slam_toolbox/points.hpp"

class Converter {
public:
  Converter() = default;

  static void PcdToBinary(const std::string &pcd_file_name,
                          const std::string &binary_file_name);

  static void BinaryToPcd(const std::string &binary_file_name,
                          const std::string &pcd_file_name);

  static void PlyToBinary(const std::string &ply_file_name,
                          const std::string &binary_file_name);

  static void BinaryToPly(const std::string &binary_file_name,
                          const std::string &ply_file_name);
  static void PcdToPly(const std::string &pcd_file_name,
                       const std::string &ply_file_name);
  static void PlyToPcd(const std::string &ply_file_name,
                       const std::string &pcd_file_name);
}; // class Converter

#endif // __SLAM_TOOLBOX_CONVERTER_H__