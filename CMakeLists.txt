cmake_minimum_required(VERSION 2.8.3)
project(stair_detector_geo)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            cv_bridge
            image_transport
            camera_info_manager
            dynamic_reconfigure
        )
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/StairDetectorGeo.cfg
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS dynamic_reconfigure
)

add_library(stair_detector_geo src/stair_detector_geo.cpp)
target_link_libraries(stair_detector_geo ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Boost_LIBRARIES})

add_executable(stair_detector_geo_node src/stair_detector_geo_node.cpp)
target_link_libraries(stair_detector_geo_node stair_detector_geo ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS 
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)