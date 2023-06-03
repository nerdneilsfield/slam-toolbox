#include "slam_toolbox/converter.hpp"
#include "slam_toolbox/binary.hpp"
#include "slam_toolbox/points.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <spdlog/spdlog.h>

void Converter::BinaryToPcd(const std::string &binary_file_name,
                            const std::string &pcd_file_name) {
  spdlog::debug("Converting binary file {} to PCD file {}", binary_file_name,
                pcd_file_name);
  BinaryIO binary_io(binary_file_name);
  binary_io.LoadFromFile();
  auto cloud = binary_io.GetPclPointCloud();
  pcl::io::savePCDFileBinary(pcd_file_name, cloud);
}

void Converter::PcdToBinary(const std::string &pcd_file_name,
                            const std::string &binary_file_name) {
  spdlog::debug("Converting PCD file {} to binary file {}", pcd_file_name,
                binary_file_name);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_name, cloud) == -1) {
    spdlog::error("Could not read file: {}", pcd_file_name);
    throw std::runtime_error("Cannot read file " + pcd_file_name);
  }
  BinaryIO binary_io(binary_file_name);
  binary_io.SetPointCloud(std::move(cloud.makeShared()));
  binary_io.DumpToFile();
}

void Converter::BinaryToPly(const std::string &binary_file_name,
                            const std::string &ply_file_name) {
  spdlog::debug("Converting binary file {} to PLY file {}", binary_file_name,
                ply_file_name);
  BinaryIO binary_io(binary_file_name);
  binary_io.LoadFromFile();
  auto cloud = binary_io.GetPclPointCloud();
  pcl::io::savePLYFileBinary(ply_file_name, cloud);
}

void Converter::PlyToBinary(const std::string &ply_file_name,
                            const std::string &binary_file_name) {
  spdlog::debug("Converting PLY file {} to binary file {}", ply_file_name,
                binary_file_name);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPLYFile<pcl::PointXYZI>(ply_file_name, cloud) == -1) {
    spdlog::error("Could not read file: {}", ply_file_name);
    throw std::runtime_error("Cannot read file " + ply_file_name);
  }
  BinaryIO binary_io(binary_file_name);
  binary_io.SetPointCloud(std::move(cloud.makeShared()));
  binary_io.DumpToFile();
}

void Converter::PcdToPly(const std::string &pcd_file_name, const std::string &ply_file_name) {
    spdlog::debug("Converting PCD file {} to PLY file {}", pcd_file_name,
                    ply_file_name);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_name, cloud) == -1) {
        spdlog::error("Could not read file: {}", pcd_file_name);
        throw std::runtime_error("Cannot read file " + pcd_file_name);
    }
    pcl::io::savePLYFileBinary(ply_file_name, cloud);
}

void Converter::PlyToPcd(const std::string &ply_file_name, const std::string &pcd_file_name) {
    spdlog::debug("Converting PLY file {} to PCD file {}", ply_file_name,
                    pcd_file_name);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPLYFile<pcl::PointXYZI>(ply_file_name, cloud) == -1) {
        spdlog::error("Could not read file: {}", ply_file_name);
        throw std::runtime_error("Cannot read file " + ply_file_name);
    }
    pcl::io::savePCDFileBinary(pcd_file_name, cloud);
}

