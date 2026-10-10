SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 3.5)  # ROS2 Humble需要至少CMake 3.5
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
endif()

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# ROS2 build system
SET(USE_ROS2 TRUE)  # ROS2 specific build
IF(USE_ROS2)
  FIND_PACKAGE(ament_cmake REQUIRED)
  FIND_PACKAGE(rclcpp REQUIRED)  # ROS2 version of roscpp
  LIST(APPEND INCLUDE_DIRECTORIES ${rclcpp_INCLUDE_DIRS})
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
                        src/omni_camera.cpp
                        src/math_utils.cpp
                        src/vision.cpp
                        src/performance_monitor.cpp
                        src/robust_cost.cpp
                        src/user_input_thread.cpp
                        src/pinhole_camera.cpp
                        src/equidistant_camera.cpp
                        src/polynomial_camera.cpp
                        src/homography.cpp
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${rclcpp_LIBRARIES})  # Link to rclcpp for ROS2

# Tests
IF(BUILD_TESTING)
  ENABLE_TESTING()
  ADD_EXECUTABLE(test_vk_common_camera test/test_camera.cpp)
  TARGET_LINK_LIBRARIES(test_vk_common_camera ${PROJECT_NAME})
  ADD_TEST(NAME test_vk_common_camera COMMAND test_vk_common_camera)

  ADD_EXECUTABLE(test_vk_common_triangulation test/test_triangulation.cpp)
  TARGET_LINK_LIBRARIES(test_vk_common_triangulation ${PROJECT_NAME})
  ADD_TEST(NAME test_vk_common_triangulation COMMAND test_vk_common_triangulation)

  ADD_EXECUTABLE(test_vk_common_patch_score test/test_patch_score.cpp)
  TARGET_LINK_LIBRARIES(test_vk_common_patch_score ${PROJECT_NAME})
  ADD_TEST(NAME test_vk_common_patch_score COMMAND test_vk_common_patch_score)
  SET_TESTS_PROPERTIES(test_vk_common_camera test_vk_common_triangulation
    test_vk_common_patch_score PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENVIRONMENT "LD_LIBRARY_PATH=${Sophus_LIBRARY_DIRS}:$ENV{LD_LIBRARY_PATH}")
ENDIF()

################################################################################
# Install
IF(NOT USE_ROS2)
  GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
  SET(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  SET(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  SET(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE )
  export( PACKAGE vikit_common )

  INSTALL(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()

IF(USE_ROS2)
  INSTALL(DIRECTORY include/ DESTINATION include)
  INSTALL(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(rclcpp)
  ament_package()
ENDIF()
