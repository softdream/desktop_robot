#include "event_manage.h"
#include "data_transport.h"
#include "data_type.h"
#include "lvgl_base.h"

#include <thread>

#define BUFFER_SIZE 4

// --------------------------------- GLOBAL DATA ------------------------------- //
event::EpollEvent event_instance;
transport::Receiver udp_srv( EmojManagementProcessPort );

char recv_buffer[BUFFER_SIZE] = { 0 };

std::string img_path = "S:../images/sleep_";

bool is_emoj_setted = false;
// ----------------------------------------------------------------------------- //

void recvCallback( int fd, void* arg )
{
        std::cout<<"callback ..."<<std::endl;

        int ret = udp_srv.read( recv_buffer, BUFFER_SIZE );
	if ( ret == 4 ) {
		sensor::EmojType emoj;
		memcpy( &emoj, recv_buffer, 4 );
		std::cout<<"emoj = "<<emoj<<std::endl;

		switch ( emoj ) {
			case sensor::SleepEmoj : {
				img_path = "S:../images/sleep_";	
				is_emoj_setted = true;
				break;
			}
			case sensor::CoeffEmoj : {
				img_path = "S:../images/coeff_";
				is_emoj_setted = true;
				break;		 
			}
			case sensor::BookEmoj : {
				img_path = "S:../images/book_";
				is_emoj_setted = true;
				break;		
			}
			case sensor::BlinkEmoj : {
				img_path = "S:../images/blink_";
                                is_emoj_setted = true;
                                break;			 
			}
			default : break;
		}

	}
	
	return;
}

void eventsRecvThread()
{
        // 1. Epoll : add a udp event
        event::Event event_recver( udp_srv.getSocketFd(), EPOLLIN, recvCallback, nullptr );
        event_recver.event |= EPOLLET;
        event_instance.addEvent( event_recver );


        // 2. Epoll : dispacher
        while ( 1 ) {
                event_instance.dispatcher();
        }
}

void imgShowThread()
{
	int cnt = 0;
	while ( 1 ) {
		if ( is_emoj_setted ) {
			std::string img_file = img_path + std::to_string( cnt ) + ".bin";
			lvgl::Lvgl::instance().showOneFrame( img_file );
		
			cnt ++;
                	if ( cnt > 8 ) {
                        	cnt = 0;
                        	sleep( 1 );
                	}
		}
		else {
			sleep(1);
		}
	}
}

int main()
{
	std::cout<<"------------- LVGL TEST ---------------"<<std::endl;

	// 1. init lvgl
	lvgl::Lvgl::instance().init();

	std::thread t1( eventsRecvThread );
        std::thread t2( imgShowThread );

        t1.join();
        t2.join();

	lvgl::Lvgl::instance().close();

	return 0;
}

