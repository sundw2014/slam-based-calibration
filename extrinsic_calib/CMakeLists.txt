cmake_minimum_required(VERSION 2.8.3)
project(exCalibration)

set(CMAKE_CXX_COMPILER "g++-4.9")
set(CMAKE_C_COMPILER "gcc-4.9")

#set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11" )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set g2o path manually since the find cmake file is not good
set ( G2O_ROOT "/home/nick/Projects/g2o")

list(APPEND CMAKE_MODULE_PATH "${G2O_ROOT}/cmake_modules")

## System dependencies are found with CMake's conventions
find_package( PCL REQUIRED COMPONENTS)
find_package( Boost COMPONENTS system REQUIRED)
find_package( OpenCV REQUIRED )
find_package( Cholmod )
find_package( libpointmatcher  REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/thirdParty/edlines
	${PROJECT_SOURCE_DIR}/thirdParty/json
	${PCL_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${CHOLMOD_INCLUDE_DIRS}
	${G2O_ROOT}
	${libpointmatcher_INCLUDE_DIRS}
	)

aux_source_directory(
	${PROJECT_SOURCE_DIR}/src
	SRC_DIRS
	)

link_directories(
	${G2O_ROOT}/lib
	${PCL_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
	)


add_definitions(
	${PCL_DEFINITIONS}
	)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
## add_dependencies(beginner_tutorials ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

# exp_* stand for experiments without library or function

# test_* stand for test of library or generalized function

# main program
add_executable(
	demo
	${PROJECT_SOURCE_DIR}/main/app_noros.cpp
	${SRC_DIRS}
	${PROJECT_SOURCE_DIR}/thirdParty/edlines/EDLineDetector.cpp
	)

target_link_libraries(
	demo
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	cholmod
	${CHOLMOD_LIBRARIES}
	g2o_core
	g2o_stuff
	g2o_solver_csparse
	g2o_csparse_extension
	g2o_types_slam3d
	g2o_types_sba
	${libpointmatcher_LIBRARIES}
	)
