#ifndef __SLAM_TOOLBOX_UTILS_H__
#define __SLAM_TOOLBOX_UTILS_H__


#include "slam_toolbox/points.hpp"

#include <string>


std::string GetFileExtension(const std::string& filename);
std::string GetFileNameWithoutExtension(const std::string& filename);

bool FileExists(const std::string& filename);

bool DirectoryExists(const std::string& directory);

bool ParentDirectoryExists(const std::string& filename);

bool MakeParentDirectory(const std::string& filename);

bool MakeDirectory(const std::string& directory);

std::vector<std::string> GetFilesInDirectory(const std::string& directory);

#endif // __SLAM_TOOLBOX_UTILS_H__