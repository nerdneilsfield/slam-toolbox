#include "slam_toolbox/binary.hpp"

#include <fstream>

#include <spdlog/spdlog.h>

BinaryIO::BinaryIO() {
  this->file_name_ = "";
  this->point_cloud_ = SimplePointCloud();
}

BinaryIO::BinaryIO(const std::string &file_name) {
  this->file_name_ = file_name;
  this->point_cloud_ = SimplePointCloud();
}

void BinaryIO::SetFileName(const std::string &file_name) {
  this->file_name_ = file_name;
}

void BinaryIO::LoadFromFile() {
  std::ifstream input_file(this->file_name_, std::ios::binary);
  if (!input_file.is_open()) {
    spdlog::error("Could not open file: {}", this->file_name_);
    throw std::runtime_error("Cannot open file " + this->file_name_);
  }
//   std::size_t size;
//   input_file.read(reinterpret_cast<char *>(&size), sizeof(size));
  this->point_cloud_.Clear();
  for (size_t i = 0; input_file.good() && !input_file.eof(); i++) {
    SimplePoint point;
    input_file.read(reinterpret_cast<char *>(&point), sizeof(point));
    this->point_cloud_.AddPoint(point);
  }
  input_file.close();
}

void BinaryIO::DumpToFile() {
  std::ofstream output_file(file_name_, std::ios::binary);
  if (!output_file.is_open()) {
    spdlog::error("Could not open file: {}", file_name_);
    throw std::runtime_error("Cannot open file " + file_name_);
  }
//   std::size_t size = this->point_cloud_.GetSize();
//   output_file.write(reinterpret_cast<char *>(&size), sizeof(size));
  for (auto point : this->point_cloud_.points) {
    output_file.write(reinterpret_cast<char *>(&point), sizeof(point));
  }
  output_file.close();
}

void BinaryIO::SetPointCloud(const SimplePointCloud &point_cloud) {
  this->point_cloud_ = point_cloud;
}

void BinaryIO::SetPointCloud(SimplePointCloud &&point_cloud) {
  this->point_cloud_= std::move(point_cloud);
}

void BinaryIO::SetPointCloud(const PointCloudPtr& cloud) {
  this->point_cloud_ = SimplePointCloud(cloud);
}

void BinaryIO::SetPointCloud(PointCloudPtr &&cloud) {
  this->point_cloud_ = SimplePointCloud(std::move(cloud));
}

SimplePointCloud BinaryIO::GetPointCloud(){ return this->point_cloud_; }

PointCloud BinaryIO::GetPclPointCloud() {
  return this->point_cloud_.ToPclPointCloud();
}
