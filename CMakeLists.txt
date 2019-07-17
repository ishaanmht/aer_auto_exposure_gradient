cmake_minimum_required(VERSION 2.8.3)
project(aer_auto_exposure_gradient)



find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
)
find_package(cmake_modules)

#find_package(OpenCV REQUIRED)
find_package(Eigen3)








catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aer_auto_exposure_gradient
  CATKIN_DEPENDS cv_bridge image_transport message_generation message_runtime roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   
  
 src/auto_exp.cpp
 src/Dehaze.cpp
 
    
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    
    )




add_executable(exp_node1 src/exp_node1.cpp)
target_link_libraries(exp_node1  ${PROJECT_NAME})
add_dependencies(exp_node1 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


