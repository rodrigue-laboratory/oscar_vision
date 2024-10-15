cmake_minimum_required(VERSION 2.8.3)
project(mimik_vision)

set(CMAKE_CXX_STANDARD 17)

find_package(realsense2 2.50.0)
find_package(Eigen3 REQUIRED)
find_package(google_cloud_cpp_vision REQUIRED)

find_package(catkin REQUIRED
  roscpp
  sensor_msgs
  image_geometry
  cv_bridge
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
    cv_bridge
  DEPENDS
    EIGEN3
    google_cloud_cpp_vision
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/realsense.cpp
  src/opencv.cpp
  src/google_cloud_ocr.cpp
  src/google_cloud_vision_conversions.cpp
)

add_executable(analyse_bacterial_colonies src/analyse_bacterial_colonies.cpp)
target_link_libraries(analyse_bacterial_colonies ${PROJECT_NAME} ${realsense2_LIBRARY} ${catkin_LIBRARIES} google-cloud-cpp::vision opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)

add_executable(annotate_images src/annotate_images.cpp)
target_link_libraries(annotate_images ${PROJECT_NAME} ${realsense2_LIBRARY} ${catkin_LIBRARIES} google-cloud-cpp::vision)

add_executable(d415_crop src/d415_crop.cpp)
target_link_libraries(d415_crop ${PROJECT_NAME} ${realsense2_LIBRARY} ${catkin_LIBRARIES} google-cloud-cpp::vision)

add_executable(d415_projection src/d415_projection.cpp)
target_link_libraries(d415_projection ${PROJECT_NAME} ${realsense2_LIBRARY} ${catkin_LIBRARIES} google-cloud-cpp::vision)

add_executable(d415_text_detection src/d415_text_detection.cpp)
target_link_libraries(d415_text_detection ${PROJECT_NAME} ${realsense2_LIBRARY} ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} google-cloud-cpp::vision)

add_executable(google_cloud_vision_quickstart src/google_cloud_vision_quickstart.cpp)
target_link_libraries(google_cloud_vision_quickstart ${PROJECT_NAME} ${realsense2_LIBRARY} ${catkin_LIBRARIES} google-cloud-cpp::vision)


install(TARGETS
  ${PROJECT_NAME}
  analyse_bacterial_colonies
  annotate_images
  d415_crop
  d415_projection
  d415_text_detection
  google_cloud_vision_quickstart
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
