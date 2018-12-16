SET(PROJECT_NAME EurocMain)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# required libraries
find_package(Eigen3 REQUIRED)
find_package(Pangolin QUIET)
find_package(OpenCV QUIET)

# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
)

# Sources files
set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/InputPacker/EurocInPacker.cpp
  ${PROJECT_SOURCE_DIR}/System/System.cpp
  ${PROJECT_SOURCE_DIR}/Parser/Parser.cpp
  ${PROJECT_SOURCE_DIR}/Frame/Frame.cpp
  ${PROJECT_SOURCE_DIR}/Feature/Feature.cpp
  ${PROJECT_SOURCE_DIR}/Tracker/OpticalFlowTracker.cpp
  ${PROJECT_SOURCE_DIR}/IMU/IMUPreintegrator.cpp
  ${PROJECT_SOURCE_DIR}/IMU/NavState.cpp

)

set(LINK_LIBS
  ${Pangolin_LIBRARIES} 
  ${OpenCV_LIBS}
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/Common
  ${PROJECT_SOURCE_DIR}/InputPacker
  ${PROJECT_SOURCE_DIR}/System
  ${PROJECT_SOURCE_DIR}/Parser
  ${PROJECT_SOURCE_DIR}/Frame
  ${PROJECT_SOURCE_DIR}/Feature
  ${PROJECT_SOURCE_DIR}/Tracker
  ${PROJECT_SOURCE_DIR}/IMU

) 

# compile main library.
add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${PROJECT_SOURCE_DIR}/EurocMain.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
