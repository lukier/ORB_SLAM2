cmake_minimum_required(VERSION 3.1)

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(ORB_SLAM2)
set(${PROJECT_NAME}_VERSION 1.0.0)

#--------------------------
# Build configuration
#--------------------------
set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DCOMPILEDWITHC11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -mtune=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native")
add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-variable)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

option(BUILD_EXAMPLES "Build examples." OFF)

include(GenerateExportHeader)

message("Building ${PROJECT_NAME}, ${CMAKE_BUILD_TYPE}, Examples: ${BUILD_EXAMPLES}")

#--------------------------
# External dependencies
#--------------------------
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)

#--------------------------
# DBoW3 Dependency
#--------------------------
set(DBOW3_SOURCES
external/DBoW3/src/BowVector.cpp
external/DBoW3/src/Database.cpp
external/DBoW3/src/DescManip.cpp
external/DBoW3/src/FeatureVector.cpp
external/DBoW3/src/QueryResults.cpp
external/DBoW3/src/quicklz.c
external/DBoW3/src/ScoringObject.cpp
external/DBoW3/src/Vocabulary.cpp
)

#--------------------------
# Our sources
#--------------------------
set(HEADERS
include/Frame.h
include/Initializer.h
include/KeyFrameDatabase.h
include/KeyFrame.h
include/LocalMapping.h
include/LoopClosing.h
include/Map.h
include/MapPoint.h
include/Optimizer.h
include/ORBextractor.h
include/ORBmatcher.h
include/ORBVocabulary.h
include/PnPsolver.h
include/Sim3Solver.h
include/System.h
include/Tracking.h
)

set(SOURCES
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
)

#--------------------------
# Building
#--------------------------
add_library(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES} ${DBOW3_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE sources external/DBoW3/src)

target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} ${G2O_LIBRARIRES})

#--------------------------
# Installation
#--------------------------
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h)
set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${${PROJECT_NAME}_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION 1)
set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_${PROJECT_NAME}_MAJOR_VERSION 1)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPATIBLE_INTERFACE_STRING ${PROJECT_NAME}_MAJOR_VERSION)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(FILES ${HEADERS} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h" DESTINATION include)
install(FILES external/DBoW3/orbvoc.dbow3 DESTINATION share/${PROJECT_NAME})

# ------------------------------------------------------------------------------
# Installation - generate version file
# ------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# ------------------------------------------------------------------------------
# Installation - Export Targets
# ------------------------------------------------------------------------------
# Config.cmake for the install tree
set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})
export(EXPORT ${PROJECT_NAME}Targets FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake") 
set(ORB_SLAM2_VOC_PATH \${ORB_SLAM2_CMAKE_DIR}/../../../share/${PROJECT_NAME})
configure_file(${PROJECT_NAME}Config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
install(EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake DESTINATION ${ConfigPackageLocation})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION ${ConfigPackageLocation})
# Config.cmake for the build tree
set(ORB_SLAM2_VOC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/DBoW3)
configure_file(${PROJECT_NAME}Config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
export(PACKAGE ${PROJECT_NAME})

#--------------------------
# Build examples
#--------------------------
if(BUILD_EXAMPLES)
    find_package(Pangolin REQUIRED)
    include_directories(${Pangolin_INCLUDE_DIRS})

    add_executable(rgbd_tum Examples/RGB-D/rgbd_tum.cc)
    target_link_libraries(rgbd_tum ${PROJECT_NAME} ${Pangolin_LIBRARIES} ${G2O_LIBRARIES})

    add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
    target_link_libraries(stereo_kitti ${PROJECT_NAME} ${Pangolin_LIBRARIES} ${G2O_LIBRARIES})

    add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
    target_link_libraries(stereo_euroc ${PROJECT_NAME} ${Pangolin_LIBRARIES} ${G2O_LIBRARIES})

    add_executable(mono_tum Examples/Monocular/mono_tum.cc)
    target_link_libraries(mono_tum ${PROJECT_NAME} ${Pangolin_LIBRARIES} ${G2O_LIBRARIES})

    add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
    target_link_libraries(mono_kitti ${PROJECT_NAME} ${Pangolin_LIBRARIES} ${G2O_LIBRARIES})

    add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
    target_link_libraries(mono_euroc ${PROJECT_NAME} ${Pangolin_LIBRARIES} ${G2O_LIBRARIES})
endif()

