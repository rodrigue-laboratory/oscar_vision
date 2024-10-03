cmake_minimum_required(VERSION 2.8.3)
project(mimik_vision)

set(CMAKE_CXX_STANDARD 17)

find_package(realsense2 2.50.0)

find_package(catkin REQUIRED
  roscpp
  sensor_msgs
  image_geometry
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_geometry
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/realsense.cpp
)

add_executable(d415_crop_image src/d415_crop_image.cpp)
target_link_libraries(d415_crop_image ${realsense2_LIBRARY} ${catkin_LIBRARIES})
# target_compile_features(open_lid_biorad_t100 PRIVATE cxx_std_17)



install(TARGETS
  # ${PROJECT_NAME}
  d415_crop_image
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
