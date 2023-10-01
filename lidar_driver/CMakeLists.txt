CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )


PROJECT( lidar_test )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17" )

ADD_DEFINITIONS( -std=c++17 )

SET( CMAKE_BUILD_TYPE Release )

# include directory
INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}	
	${CMAKE_SOURCE_DIR}/include/
	${CMAKE_SOURCE_DIR}/include/gs2_include/
)

# lib directories of gs2 lidar
LINK_DIRECTORIES( ${CMAKE_SOURCE_DIR}/lib/ )

# opencv
FIND_PACKAGE( OpenCV REQUIRED )

# include directory of Opencv
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

# link directories of opencv
LINK_DIRECTORIES( ${OpenCV_LIBRARIES_DIRS} )

# package of eigen3
FIND_PACKAGE( Eigen3 )

# include directory of Eigen3
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

ADD_EXECUTABLE( ${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp )

TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ydlidar_driver.a pthread )

TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${OpenCV_LIBS} )
