#include "emoj_manage.h"
#include "event_manage.h"
#include "data_transport.h"
#include "data_type.h"

#include <thread>

#define BUFFER_SIZE 12

// --------------------------------- GLOBAL DATA ------------------------------- //
event::EpollEvent event_instance;
transport::Receiver udp_srv( EmojManagementProcessPort );

emoj::EmojFactory emoj_factory;

char recv_buffer[BUFFER_SIZE] = { 0 };
// ---------------------------------------------------------------------------- //

void happyThread()
{
	auto happy = emoj_factory.createEmoj<emoj::Happy>();
        happy->show();
}

void blinkThread()
{
	auto blink = emoj_factory.createEmoj<emoj::Blink>();
        blink->show();
}

void sleepThread()
{
	auto sleep = emoj_factory.createEmoj<emoj::Sleep>();
        sleep->show();
}

void coeffThread()
{
	auto coeff = emoj_factory.createEmoj<emoj::Coeff>();
        coeff->show();
}

void bookThread()
{
	auto book = emoj_factory.createEmoj<emoj::Book>();
        book->show();
}

void recvCallback( int fd, void* arg )
{
        std::cout<<"callback ..."<<std::endl;

        int ret = udp_srv.read( recv_buffer, BUFFER_SIZE );
	if ( ret > 0 ) {
		sensor::EmojType emoj;

		memset( recv_buffer, 0, sizeof( sensor::EmojType ) );
		memcpy( &emoj, recv_buffer, sizeof( sensor::EmojType ) );
		
		lvgl::Lvgl::instance().stopImgShow();
		sleep( 1 );

		switch ( emoj ) {
			case sensor::HappyEmoj : {
				std::thread t( happyThread );
       				t.join();				
			}
			case sensor::BlinkEmoj : {
				std::thread t( blinkThread );              
                                t.join();		 
			}
			case sensor::SleepEmoj : {
				std::thread t( sleepThread );              
                                t.join();			 
			}
			case sensor::CoeffEmoj : {
				std::thread t( coeffThread );              
                                t.join();		 
			}
			case sensor::BookEmoj : {
				std::thread t( bookThread );              
                                t.join();		
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


int main()
{
	std::cout<<"------------- LVGL TEST ---------------"<<std::endl;

	lvgl::Lvgl::instance().init();
	
	eventsRecvThread();


	return 0;
}

