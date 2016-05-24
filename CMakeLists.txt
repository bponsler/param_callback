cmake_minimum_required(VERSION 2.8.3)
project(param_callback)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Boost REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# List of all necessary header directories
include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(LIBS
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
)


############################################################
# Define libraries
#

#### main library
add_library(${PROJECT_NAME}
  src/ParamCallback.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBS})
set(LIBS ${PROJECT_NAME} ${LIBS})


############################################################
# Define tests
#

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(param_callback_tests test/param_callback_tests.cpp)
  target_link_libraries(param_callback_tests ${LIBS})
endif()