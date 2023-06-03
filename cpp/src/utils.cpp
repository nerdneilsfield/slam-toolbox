#include "slam_toolbox/utils.hpp"

#include <algorithm>
#include <filesystem>

#include <spdlog/spdlog.h>

std::string GetFileExtension(const std::string &filename) {
  std::string extension = filename.substr(filename.find_last_of(".") + 1);
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  return extension;
}

std::string GetFileNameWithoutExtension(const std::string &filename) {
  std::string name = filename.substr(0, filename.find_last_of("."));
  auto parent_directory = name.substr(0, name.find_last_of("/"));

  if (parent_directory.empty()) {
    return name;
  } else {
    return name.substr(name.find_last_of("/") + 1);
  }
}

bool FileExists(const std::string &filename) {
  if (!std::filesystem::exists(filename))
    return false;
  if (!std::filesystem::is_regular_file(filename))
    return false;
  return true;
}

bool DirectoryExists(const std::string &directory) {

  if (!std::filesystem::exists(directory))
    return false;
  if (!std::filesystem::is_directory(directory))
    return false;
  return true;
}

bool ParentDirectoryExists(const std::string &filename) {
  std::string parent_directory = filename.substr(0, filename.find_last_of("/"));
  spdlog::debug("parent_directory: {}", parent_directory);
  if(parent_directory == filename)
    return true;
  if(parent_directory.empty())
    return true;
  if (!std::filesystem::exists(parent_directory))
    return false;
  if (!std::filesystem::is_directory(parent_directory))
    return false;
  return true;
}

bool MakeParentDirectory(const std::string &filename) {
  std::string parent_directory = filename.substr(0, filename.find_last_of("/"));
  if (!std::filesystem::create_directory(parent_directory))
    return false;
  return true;
}

bool MakeDirectory(const std::string &directory) {
  if (!std::filesystem::create_directory(directory))
    return false;
  return true;
}

std::vector<std::string> GetFilesInDirectory(const std::string &directory) {
  std::vector<std::string> files;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    if (std::filesystem::is_regular_file(entry.path())) {
      files.push_back(entry.path().string());
    }
  }
  return files;
}