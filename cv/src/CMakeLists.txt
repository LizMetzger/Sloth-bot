cmake_minimum_required(VERSION 3.8)
project(rung_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_INCLUDE_PATH "/home/liz/nuws/install/lib/librealsense2")
set(CMAKE_INCLUDE_PATH "/home/liz/nuws/install")

# set(CMAKE_INCLUDE_PATH "/home/liz/nuws")
add_executable(rung_detection rung_detection.cpp)

find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED)
find_package(realsense2 REQUIRED)

set(DEPENDENCIES realsense2 ${OpenCV_LIBS} ${realsense2_LIBS})
# set(DEPENDENCIES realsense2 ${OpenCV_LIBS})

find_library(DYNAMIXEL_LIB dxl_x64_cpp PATH usr/local/lib)
# find_library(librealsense2_LIBS librealsense2 PATH /home/liz/nuws/install/lib)

link_directories(/home/liz/nuws/install/lib/librealsense2.so)
include_directories(/home/liz/winter_project/Sloth-bot/cv/include/vision)
include_directories(rung_detection ${OpenCV_INCLUDE_DIRS})

target_link_libraries(rung_detection ${OpenCV_LIBS})
# target_link_libraries(rung_detection ${DEPENDENCIES})
target_link_libraries(rung_detection ${DYNAMIXEL_LIB} ${DEPENDENCIES} ${realsense2_LIBRARY})
