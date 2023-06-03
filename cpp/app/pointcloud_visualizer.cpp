#include "slam_toolbox/binary.hpp"
#include "slam_toolbox/utils.hpp"
#include "slam_toolbox/visualizer.hpp"

#include <CLI/CLI.hpp>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <spdlog/spdlog.h>

int main(int argc, char *argv[]) {
  CLI::App app{"SLAM Toolbox: Visualizer"};

  std::string input_path;
  std::string color;
  app.add_option("-i,--input", input_path, "Path to the input files");
  app.add_option("-c,--color", color, "Color of the point cloud");

  bool verbose = false;
  app.add_flag("-v,--verbose", verbose, "Verbose mode");
  bool direcotry_mode = false;
  app.add_flag("-d,--directory-mode", direcotry_mode, "Directory mode");

  CLI11_PARSE(app, argc, argv);

  if (verbose) {
    spdlog::set_level(spdlog::level::debug);
  } else {
    spdlog::set_level(spdlog::level::info);
  }

  if (direcotry_mode) {
    spdlog::info("Directory mode enabled");
  } else {
    spdlog::info("File mode enabled");
  }

  spdlog::info("Input path: {}", input_path);
  spdlog::info("Color: {}", color);

  Visualizer visualizer("point_cloud", color);

  visualizer.SpinOnce(1000);

  if (!direcotry_mode) {
    auto input_type = GetFileExtension(input_path);
    if (input_type == "pcd") {
      PointCloudPtr point_cloud(new PointCloud);
      pcl::io::loadPCDFile(input_path, *point_cloud);
      visualizer.UpdatePointCloud(point_cloud, "point_cloud", color);
    } else if (input_type == "ply") {
      PointCloudPtr point_cloud(new PointCloud);
      pcl::io::loadPLYFile(input_path, *point_cloud);
      visualizer.UpdatePointCloud(point_cloud, "point_cloud", color);
    } else if (input_type == "bin") {
      BinaryIO binary_io(input_path);
      binary_io.LoadFromFile();
      visualizer.UpdatePointCloud(binary_io.GetPointCloud(), "point_cloud",
                                  color);
    } else {
      spdlog::error("Unknown file type");
    }
  } else {
    auto file_paths = GetFilesInDirectory(input_path);
    int32_t id = 0;
    for (const auto &file_path : file_paths) {
      auto input_type = GetFileExtension(file_path);
      spdlog::debug("Processing file: {} with id: {}", file_path, id);
      if (input_type == "pcd") {
        PointCloudPtr point_cloud(new PointCloud);
        pcl::io::loadPCDFile(file_path, *point_cloud);
        visualizer.UpdatePointCloud(point_cloud, "point_cloud", color, id);
      } else if (input_type == "ply") {
        PointCloudPtr point_cloud(new PointCloud);
        pcl::io::loadPLYFile(file_path, *point_cloud);
        visualizer.UpdatePointCloud(point_cloud, "point_cloud", color, id);
      } else if (input_type == "bin") {
        BinaryIO binary_io(file_path);
        binary_io.LoadFromFile();
        visualizer.UpdatePointCloud(binary_io.GetPointCloud(), "point_cloud",
                                    color, id);
      } else {
        spdlog::error("Unknown file type");
      }
      id++;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

    while (!visualizer.WasStopped()) {
        visualizer.SpinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}