cmake_minimum_required(VERSION 3.0.2)
project(base-particle-filtering)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Pkg_config is required for bfl library
find_package(PkgConfig)

pkg_check_modules(BFL REQUIRED orocos-bfl)

message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

include_directories(${BFL_INCLUDE_DIRS}/bfl)
link_directories(${BFL_LIBRARY_DIRS})
link_directories(/opt/ros/melodic/lib)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base-particle-filtering
#  CATKIN_DEPENDS bfl geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

# Executables
add_executable(pf_mobile_node src/node_wall_pf.cpp src/nonlinearSystemPdf.cpp src/nonlinearMeasurementPdf.cpp src/mobile_robot.cpp src/nonlinearanalyticconditionalgaussianmobile.cpp src/customParticleFilter.cpp)
add_executable(pf_fullstate_mobile_node src/node_fullState_pf.cpp src/nonlinearSystemPdf.cpp src/stateMeasurementPdf.cpp src/mobile_robot_fullstate.cpp src/nonlinearanalyticconditionalgaussianmobile.cpp src/customParticleFilter.cpp)
add_executable(pf_fullstate_planaragent_node src/node_planaragent_pf.cpp src/planarSystemPdf.cpp src/stateMeasurementPdf.cpp src/planar_agent.cpp src/planaragentconditionalgaussian.cpp src/customParticleFilter.cpp)

# Libraries to be linked to executables
target_link_libraries(pf_mobile_node
  ${catkin_LIBRARIES}
  orocos-bfl 
  ${MATRIX_LIBS} 
  ${RNG_LIBS}  
  ${GINAC_LIBS}
  Eigen3::Eigen
)
target_link_libraries(pf_fullstate_mobile_node
  ${catkin_LIBRARIES}
  orocos-bfl 
  ${MATRIX_LIBS} 
  ${RNG_LIBS}  
  ${GINAC_LIBS}
  Eigen3::Eigen
)

target_link_libraries(pf_fullstate_planaragent_node
  ${catkin_LIBRARIES}
  orocos-bfl 
  ${MATRIX_LIBS} 
  ${RNG_LIBS}  
  ${GINAC_LIBS}
  Eigen3::Eigen
)

