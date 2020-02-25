cmake_minimum_required(VERSION 2.8)

PROJECT(StereoStitching)

add_definitions(-std=c++11 -w)

#openMP
FIND_PACKAGE(OpenMP REQUIRED)

#pcl
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#eigen
FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE -std=c++11")

#opencv
find_package(OpenCV REQUIRED)	
include_directories(${OpenCV_INCLUDE_DIRS})

#libtiff
SET(LIBTIFF_LIBRARY /usr/lib/x86_64-linux-gnu/libtiff.so.5)

#vlfeat
SET(VLFEAT_LIBRARY ${PROJECT_SOURCE_DIR}/vlfeat-0.9.14/bin/glnxa64/libvl.so)

aux_source_directory(./Debugger DEBUGGER_SRC)
aux_source_directory(./Feature FEATURE_SRC)
aux_source_directory(./Mesh MESH_SRC)
aux_source_directory(./Stitching STITCHING_SRC)
aux_source_directory(./Util UTIL_SRC)
aux_source_directory(./CasualUtil CASUALUTIL_SRC)
aux_source_directory(./CommonUtil COMMONUTIL_SRC)
aux_source_directory(./OurUtil OURUTIL_SRC)
aux_source_directory(./NFeature NFeatureL_SRC)

add_executable(zhang_15_stitching Configure.h casual_stitching.cpp ${DEBUGGER_SRC} ${FEATURE_SRC} ${MESH_SRC} ${STITCHING_SRC} ${UTIL_SRC} ${COMMONUTIL_SRC} ${CASUALUTIL_SRC})
target_link_libraries(zhang_15_stitching ${OpenCV_LIBS} ${VLFEAT_LIBRARY} ${LIBTIFF_LIBRARY})

add_executable(stitch_based_on_3D Configure.h our_stitching.cpp ${DEBUGGER_SRC} ${FEATURE_SRC} ${MESH_SRC} ${STITCHING_SRC} ${UTIL_SRC} ${COMMONUTIL_SRC} ${CASUALUTIL_SRC} ${OURUTIL_SRC})
target_link_libraries (stitch_based_on_3D ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VLFEAT_LIBRARY} ${LIBTIFF_LIBRARY})
