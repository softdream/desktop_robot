CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )

PROJECT( behavior_test )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17" )

ADD_DEFINITIONS( -std=c++17 )

SET( CMAKE_BUILD_TYPE Release )

# find required eigen3
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

# include directory
INCLUDE_DIRECTORIES(
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include/
)

# third party include directory
INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}/third_party/include/
)

# third party libraries directory


#ADD_DEFINITIONS( -lwiringPi )

#ADD_EXECUTABLE( ${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp )

# third party source files 
AUX_SOURCE_DIRECTORY( ${CMAKE_SOURCE_DIR}/third_party/src/ src_files )

AUX_SOURCE_DIRECTORY( ${CMAKE_SOURCE_DIR}/ src_files )

ADD_EXECUTABLE( ${PROJECT_NAME} ${src_files} )

TARGET_LINK_LIBRARIES( ${PROJECT_NAME} pthread )
