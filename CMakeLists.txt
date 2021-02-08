cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)

## add executables

add_executable(calibrate
   src/camera_calibration.cpp
)

add_executable(demo
   src/demo_full_example.cpp
)

add_executable(match_template
   src/match_template.cpp
)


## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
   ##src/camera_calibration.cpp
   src/extrincic_calibration.cpp
   src/findRobot.cpp
   ##src/digit_template_match.cpp
   src/process_map.cpp
)

add_library(dubins SHARED
   src/dubins.cpp
)

add_library(polyclipping SHARED
   src/clipper/clipper.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES} 
  dubins
  polyclipping
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}  
)

target_link_libraries(demo
  ${OpenCV_LIBRARIES}
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}
)

target_link_libraries(match_template
  ${OpenCV_LIBRARIES}  
)
 

