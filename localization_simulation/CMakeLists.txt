CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )

PROJECT( localize_test )

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

#ADD_DEFINITIONS( -lwiringPi )

ADD_EXECUTABLE( ${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp )

TARGET_LINK_LIBRARIES( ${PROJECT_NAME} pthread )

