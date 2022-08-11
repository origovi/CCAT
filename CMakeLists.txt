cmake_minimum_required(VERSION 3.0.2)
project(ccat)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

message(STATUS "Welcome to CCAT :)")
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs as_msgs shrd_msgs pcl_ros tf tf_conversions eigen_conversions image_transport cv_bridge dynamic_reconfigure)
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
else()
  message(WARNING "OPENMP NOT FOUND, multithread disabled")
endif()

add_service_files(
  FILES
  CalibReq.srv
)

generate_messages(
  DEPENDENCIES
  as_msgs
  std_msgs
  shrd_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Extrinsics.cfg
  cfg/TimeDiff.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS OpenMP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_exec
  src/main.cpp
)

add_library(${PROJECT_NAME}
  src/Manager.cpp
  src/modules/Preproc/Preproc.cpp
  src/modules/Preproc/PreprocVis.cpp
  src/modules/Matcher/Matcher.cpp
  src/modules/Matcher/MatcherVis.cpp
  src/modules/Merger/Merger.cpp
  src/modules/Tracker/Tracker.cpp
  src/modules/Tracker/TrackerVis.cpp
  src/structures/Cone.cpp
  src/structures/Point.cpp
  src/structures/Observation.cpp
  src/structures/ConeUpdate.cpp
  src/structures/Params.cpp
  src/utils/KDTree.cpp
  src/utils/Visualization.cpp
  src/utils/Time.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenMP_LIBS})
target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME})

# Mandatory for parallelism (OpenMP)
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})