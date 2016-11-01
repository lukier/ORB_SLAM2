cmake_minimum_required(VERSION 3.1)

project(ORB_SLAM2)
set(${PROJECT_NAME}_VERSION 1.0.0)

#--------------------------
# Build configuration
#--------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DCOMPILEDWITHC11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -mtune=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native")
add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-variable)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

option(BUILD_EXAMPLES "Build examples." OFF)

include(GenerateExportHeader)

#--------------------------
# External dependencies
#--------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

find_package(DLib REQUIRED)
find_package(DBoW2 REQUIRED)
include_directories(${DBoW2_INCLUDE_DIR})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#--------------------------
# Our sources
#--------------------------
set(HEADERS
include/dbow_extensions.h
include/FrameDrawer.h
include/Frame.h
include/Initializer.h
include/KeyFrameDatabase.h
include/KeyFrame.h
include/LocalMapping.h
include/LoopClosing.h
include/MapDrawer.h
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
include/Viewer.h
)

set(SOURCES
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

#--------------------------
# Building
#--------------------------
add_library(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE sources)
    
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${DBoW2_LIBRARIES}
${DLib_LIBRARIES}
${G2O_LIBRARIRES}
${Pangolin_LIBRARIES}
)

#--------------------------
# Installation
#--------------------------
generate_export_header(${PROJECT_NAME})
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
export(EXPORT ${PROJECT_NAME}Targets FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake") 
configure_file(${PROJECT_NAME}Config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake DESTINATION ${ConfigPackageLocation})
install(FILES ${PROJECT_NAME}Config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION ${ConfigPackageLocation})
export(PACKAGE ${PROJECT_NAME})

#--------------------------
# Build examples
#--------------------------
if(BUILD_EXAMPLES)
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

