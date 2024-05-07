#MIT License
#
#Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.


cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

project(TrajLO LANGUAGES C CXX)

if (NOT CXX_MARCH)
    set(CXX_MARCH native)
endif ()

add_subdirectory(thirdparty)

include_directories(include)

add_definitions("-D SOPHUS_USE_BASIC_LOGGING")
include_directories(thirdparty/Sophus)

add_executable(trajlo run_trajlo.cpp
        src/io/data_loader.cpp
        src/utils/config.cpp
        src/gui/visualizer.cpp
        src/gui/glad.c
        src/core/odometry.cpp
        src/core/map_manager.cpp
)
target_link_libraries(trajlo PUBLIC
        yaml-cpp::yaml-cpp
        Eigen3::Eigen
        ${TBB_LIBRARIES}
        rosbag
        pthread
        imgui glm::glm
        PRIVATE tsl::robin_map
        TBB::tbb
)