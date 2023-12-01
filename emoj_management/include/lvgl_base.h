#ifndef __LVGL_BASE_H
#define __LVGL_BASE_H

#ifdef __cplusplus
extern "C" {
#endif
        #include "lvgl.h"
        #include "fbdev.h"
        #include "evdev.h"

#ifdef __cplusplus
} /* extern "C" */
#endif

#include <unistd.h>
#include <iostream>

// display buffer size
#define LV_BUF_SIZE 115200              // 240 * 240 * 2

#define IMG_WIDTH 240 
#define IMG_HEIGTH 240
#define IMGS_NUM 7
#define IMG_FILE_SIZE 115200

namespace lvgl
{

class Lvgl
{
private:
	Lvgl()
	{
	
	}

	~Lvgl()
	{
	
	}

	static Lvgl instance_;

public:	
	static Lvgl& instance()
	{
		return instance_;
	}	

	void init()
	{
		lv_init();

		// linux frame buffer device init
		fbdev_init();

		// touch pointer device init
		evdev_init();

		// initialize `disp_buf` with the display buffer(s)
        	lv_disp_buf_init( &disp_buf_, lvbuf1_, lvbuf2_, LV_BUF_SIZE );

		// initialize and register a display driver
        	lv_disp_drv_t disp_drv;
        	lv_disp_drv_init( &disp_drv );
        	disp_drv.flush_cb = fbdev_flush;        // flushes the internal graphical buffer to the frame buffer
        	disp_drv.buffer = &disp_buf_;            // set teh display buffere reference in the driver
        	lv_disp_drv_register( &disp_drv );
	
		// initialize and register a pointer device driver
        	lv_indev_drv_t indev_drv;
        	lv_indev_drv_init( &indev_drv );
	        indev_drv.type = LV_INDEV_TYPE_POINTER;
        	indev_drv.read_cb = evdev_read;    // defined in lv_drivers/indev/evdev.h
	        lv_indev_drv_register( &indev_drv );
	
		// initialize the file system
                lv_port_fs_init();
	
		// init the image
		img_obj_ = lv_img_create( lv_scr_act(), NULL );
	}

	void showOneFrame( const std::string& file_name, const int perid = 200 ) 
	{
		lv_fs_res_t res = lv_fs_open( &f_, file_name.c_str(), LV_FS_MODE_RD );
                if ( res != LV_FS_RES_OK ) {
                	std::cerr<<"Failed to Open the file : "<<file_name<<std::endl;
                        return ;
                }

		lv_fs_seek( &f_, 4 );
		memset( buffer_, 0, sizeof(buffer_) );
		res = lv_fs_read( &f_, (uint8_t*)buffer_, IMG_FILE_SIZE, NULL );
		if ( res != LV_FS_RES_OK ) {
                        std::cerr<<"Failed to Read the file ."<<std::endl;
                        return ;
                }

		lv_img_dsc_t img;
                img.header.always_zero = 0;
                img.header.w = IMG_WIDTH;
                img.header.h = IMG_HEIGTH;
                img.data_size = IMG_FILE_SIZE;
                img.header.cf = LV_IMG_CF_TRUE_COLOR;

                img.data = (const uint8_t*)buffer_;

                lv_img_set_src( img_obj_, &img );
                lv_obj_align( img_obj_, NULL, LV_ALIGN_CENTER, 0, 0 );
		
		lv_fs_close( &f_ );

		lv_tick_inc( perid );
                lv_task_handler();
                usleep( perid * 1000 );
	}

	void close()
	{
		if ( !img_obj_ ) delete img_obj_;
	}

private:
	// variable to store the display buffers
	lv_disp_buf_t disp_buf_;
	
	// buffer(s). The second buffer is optional
	lv_color_t lvbuf1_[LV_BUF_SIZE];
	lv_color_t lvbuf2_[LV_BUF_SIZE];

	// buffers for image files
	uint8_t buffer_[IMG_FILE_SIZE];
	
	// images show stop flag 
	bool img_show_stop_flag = false;

	// file system
	lv_fs_file_t f_;

	// image
	lv_obj_t* img_obj_ = nullptr;
};


Lvgl Lvgl::instance_;

}

#endif
