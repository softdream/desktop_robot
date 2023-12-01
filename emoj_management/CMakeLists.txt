CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )

PROJECT ( lvgl_test )

INCLUDE_DIRECTORIES(
        ${CMAKE_SOURCE_DIR}/include
	
	${CMAKE_SOURCE_DIR}/lvgl/include
	${CMAKE_SOURCE_DIR}/lvgl/include/src
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_draw
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_core
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_font
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_gpu
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_hal
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_misc
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_themes
	${CMAKE_SOURCE_DIR}/lvgl/include/src/lv_widgets
	${CMAKE_SOURCE_DIR}/lvgl/include/porting
	${CMAKE_SOURCE_DIR}/lvgl/include/lv_drivers
	${CMAKE_SOURCE_DIR}/lvgl/include/lv_drivers/display
	${CMAKE_SOURCE_DIR}/lvgl/include/lv_drivers/indev
)

# find required eigen3
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )



LINK_DIRECTORIES( ${CMAKE_SOURCE_DIR}/lvgl/lib )

AUX_SOURCE_DIRECTORY( ${CMAKE_SOURCE_DIR} src_files )

ADD_EXECUTABLE( ${PROJECT_NAME} ${src_files} )

TARGET_LINK_LIBRARIES( ${PROJECT_NAME} liblvgl.a )

