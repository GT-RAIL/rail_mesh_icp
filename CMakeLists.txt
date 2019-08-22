cmake_minimum_required(VERSION 2.8.3)
project(rail_mesh_icp)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED 1.8 REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  pcl_ros
  pcl_conversions
  tf2_ros
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
  ICPMatch.srv
  TemplateMatch.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LINK_LIBS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)

# compiles cpp nodes
add_executable(icp_matcher_node src/icp_matcher_node.cpp src/ICPMatching.cpp)
add_dependencies(icp_matcher_node ${catkin_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(icp_matcher_node ${LINK_LIBS})

add_executable(template_matcher_node src/template_matcher_node.cpp src/TemplateMatching.cpp)
add_dependencies(template_matcher_node ${catkin_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS})
target_link_libraries(template_matcher_node ${LINK_LIBS})

add_executable(mesh_sampler_node tools/mesh_sampler_node.cpp)
add_dependencies(mesh_sampler_node ${catkin_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(mesh_sampler_node ${LINK_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS icp_matcher_node mesh_sampler_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
