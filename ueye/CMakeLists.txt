cmake_minimum_required(VERSION 2.8.3)
project(ueye)

#find the cv support and set HAVE_OPENCV_H for the conditional compilation
FIND_PACKAGE(OpenCV REQUIRED core highgui)

if (OpenCV_FOUND)
  if (${OpenCV_VERSION_MAJOR} GREATER 1)
    if (${OpenCV_VERSION_PATCH} GREATER 8)
      message("-- [INFO] Found OpenCV support")
      ADD_DEFINITIONS(-DHAVE_OPENCV_H)
      SET(USE_CV true)
    else (${OpenCV_VERSION_PATCH} GREATER 8)
      message(" minimum requried OpenCV version is 2.4.8")
    endif (${OpenCV_VERSION_PATCH} GREATER 8)
  else(${OpenCV_VERSION_MAJOR} GREATER 1)
    message("[WARN] OpenCV support not installed. Minimum 2.0 version required")
    message("[WARN] Current version ${OpenCV_VERSION_MAJOR}")
  endif(${OpenCV_VERSION_MAJOR} GREATER 1)
else(OpenCV_FOUND)
    message("[WARN] OpenCV support not installed. Minimum 2.0 version required")
endif(OpenCV_FOUND)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)
catkin_destinations()

FIND_PACKAGE(iriutils REQUIRED)

# Declare a catkin package
catkin_package()

SET(sources src/ueye_camera.cpp src/ueye_exceptions.cpp)
SET(headers include/ueye_camera.h include/ueye_exceptions.h)
SET(CMAKE_BUILD_TYPE release)

# try to find the ueye library
FIND_PATH(ueye_camera_INCLUDE_DIR ueye.h /usr/include /usr/local/include)

FIND_LIBRARY(ueye_camera_LIBRARY
    NAMES libueye_api.so
    PATHS /usr/lib /usr/local/lib)

IF (ueye_camera_INCLUDE_DIR AND ueye_camera_LIBRARY)
   SET(ueye_camera_ready TRUE)
ENDIF (ueye_camera_INCLUDE_DIR AND ueye_camera_LIBRARY)

IF (ueye_camera_ready)
    MESSAGE(STATUS "The system is ueye_camera capable")
    # add the necessary include directories
    INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR} ${ueye_camera_INCLUDE_DIR} .)

    # create the shared library
    ADD_LIBRARY(ueye_camera SHARED ${sources})

    TARGET_LINK_LIBRARIES(ueye_camera ${iriutils_LIBRARY} ${ueye_camera_LIBRARY})

    # link necessary libraries
    INSTALL(TARGETS ueye_camera
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/iridrivers
    ARCHIVE DESTINATION lib/iridrivers)

    INSTALL(FILES ${headers} DESTINATION include/iridrivers)
    INSTALL(FILES ../Findueye_camera.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

    #ADD_SUBDIRECTORY(src)
ELSE (ueye_camera_ready)
   MESSAGE(FATAL_ERROR "The system is not ueye_camera capable")
ENDIF (ueye_camera_ready)

#set(UEYE_LIBRARY_PATH /usr/lib)
find_library(UEYE_LIBRARY ueye_api)
set(USE_UNOFFICIAL_UEYE_DRIVERS FALSE)
if(NOT UEYE_LIBRARY)
    message(STATUS "libueye_api was not found in system's library path")
    #include(cmake_modules/DownloadUEyeDrivers.cmake)
    include(cmake_modules/DownloadUEyeDriversUnofficial.cmake)
    set(UEYE_DRIVER_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
    	if(UEYE_DRIVER_DIR)
    	    message(STATUS "Setting UEYE_DRIVER_DIR to: ${UEYE_DRIVER_DIR}")
  	else()
    	    set(UEYE_DRIVER_DIR ${CMAKE_CURRENT_BINARY_DIR})
    	    message(STATUS "Failed to set UEYE_DRIVER_DIR to: CATKIN_DEVEL_PREFIX/CATKIN_PACKAGE_SHARE_DESTINATION, defaulting to CMAKE_CURRENT_BINARY_DIR: ${UEYE_DRIVER_DIR}")
  	endif()
    download_ueye_drivers(UEYE_LIBRARY UEYE_INCLUDE_DIR ${UEYE_DRIVER_DIR})
    message(STATUS "libueye_api library: ${UEYE_LIBRARY}")
    message(STATUS "libueye_api include: ${UEYE_INCLUDE_DIR}")
    set(USE_UNOFFICIAL_UEYE_DRIVERS TRUE)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(ueye_node src/ueye_node.cpp)
target_link_libraries(ueye_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ueye_camera)
add_dependencies(ueye_node ueye_cpp)

