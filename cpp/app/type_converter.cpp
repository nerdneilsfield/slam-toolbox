#include "slam_toolbox/converter.hpp"
#include "slam_toolbox/points.hpp"
#include "slam_toolbox/utils.hpp"

#include <CLI/CLI.hpp>
#include <cstdlib>
#include <spdlog/spdlog.h>

void ConvertFile(const std::string &input_file, const std::string &output_file,
                 const std::string &output_type) {
  auto input_type = GetFileExtension(input_file);

  if (input_type == output_type) {
    spdlog::error("Input type and output type are the same");
    std::exit(1);
  }

  if (input_type == "bin" && output_type == "pcd") {
    Converter::BinaryToPcd(input_file, output_file);
    return;
  }
  if (input_type == "bin" && output_type == "ply") {
    Converter::BinaryToPly(input_file, output_file);
    return;
  }
  if (input_type == "pcd" && output_type == "bin") {
    Converter::PcdToBinary(input_file, output_file);
    return;
  }
  if (input_type == "ply" && output_type == "bin") {
    Converter::PlyToBinary(input_file, output_file);
    return;
  }
  if (input_type == "pcd" && output_type == "ply") {
    Converter::PcdToPly(input_file, output_file);
    return;
  }
  if (input_type == "ply" && output_type == "pcd") {
    Converter::PlyToPcd(input_file, output_file);
    return;
  }
  spdlog::warn("Unknown conversion type");
}

int main(int argc, char **argv) {
  CLI::App app{"SLAM Toolbox: type converter"};

  std::string input_file;
  app.add_option("-i,--input", input_file, "Input file or directory");
  std::string output_file;
  app.add_option("-o,--output", output_file, "Output file or directory");
  std::string output_type;
  app.add_option("-t,--type", output_type, "Output type: [pcd, ply, bin]");

  bool directory_mode = false;
  bool verbose = false;

  app.add_flag("-d,--directory-mode", directory_mode, "Directory mode");
  app.add_flag("-v,--verbose", verbose, "Verbose mode");

  CLI11_PARSE(app, argc, argv);

  if (verbose) {
    spdlog::info("Set log level to debug");
    spdlog::set_level(spdlog::level::debug);
  } else {
    spdlog::info("Set log level to info");
    spdlog::set_level(spdlog::level::info);
  }

  if (directory_mode) {
    spdlog::info("Directory mode enabled");
  } else {
    spdlog::info("File mode enabled");
  }

  spdlog::info("Input file/directory: {}", input_file);
  spdlog::info("Output file/direcotry: {}", output_file);
  spdlog::info("Output type: {}", output_type);

  if (directory_mode) {
    if (!DirectoryExists(input_file)) {
      spdlog::error("Input directory does not exist");
      return 1;
    }

    if (!ParentDirectoryExists(output_file)) {
      spdlog::error("Parent Directory of output directory does not exist");
      return 1;
    }

    if (!DirectoryExists(output_file)) {
      spdlog::warn("Output directory does not exist");
      spdlog::info("Creating output directory");
      if (!MakeDirectory(output_file)) {
        spdlog::error("Failed to create output directory");
        return 1;
      }
    }
  } else {
    if (!FileExists(input_file)) {
      spdlog::error("Input file does not exist");
      return 1;
    }

    if (!ParentDirectoryExists(output_file)) {
      spdlog::error("Parent Directory of output file does not exist");
      return 1;
    }
  }

  if (output_type != "ply" && output_type != "pcd" && output_type != "bin") {
    spdlog::error("Invalid output type");
    return 1;
  }

  if (directory_mode) {
    spdlog::info("Converting all files in directory");
    std::vector<std::string> files = GetFilesInDirectory(input_file);
    for (const auto &file : files) {
      spdlog::info("Converting file: {}", file);
      auto file_without_ext = GetFileNameWithoutExtension(file);
      std::string output_file_path =
          output_file + "/" + file_without_ext + "." + output_type;
      ConvertFile(file, output_file_path, output_type);
    }
  } else {
    spdlog::info("Converting file {} to {}", input_file, output_file);
    ConvertFile(input_file, output_file, output_type);
  }
}
