find_package(PCL REQUIRED COMPONENTS common io visualization QUIET)

add_library(slam_toolbox SHARED src/binary.cpp src/converter.cpp src/points.cpp src/utils.cpp src/visualizer.cpp)
target_include_directories(slam_toolbox PUBLIC include)
target_link_libraries(slam_toolbox PUBLIC ${PCL_LIBRARIES} spdlog::spdlog)

add_executable(type_converter app/type_converter.cpp)
target_link_libraries(type_converter PRIVATE slam_toolbox CLI11::CLI11)
add_executable(pointcloud_visualizer app/pointcloud_visualizer.cpp)
target_link_libraries(pointcloud_visualizer PRIVATE slam_toolbox CLI11::CLI11)
