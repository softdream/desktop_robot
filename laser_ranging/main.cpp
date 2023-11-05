#include "ranging_sensor.h"
#include "event_manage.h"

#include "time_stamp.h"

sensor::RangingSensor ranging_sensor;

void recvCallback( int fd, void* arg )
{
	std::cout<<"callback ..."<<std::endl;

	auto stamp = stamp::TimeStamp::getCurrentTimeMs();
        std::cout<<"stamp ================== "<<stamp<<std::endl;

	ranging_sensor.getMeasuredVal<float>();
}

int main()
{
	std::cout<<"----------- LASER RANGING TSET --------------"<<std::endl;

	ranging_sensor.init();

	event::EpollEvent event_instance;

	event::Event uart_event( ranging_sensor.getFd(), EPOLLIN, recvCallback, nullptr );
	uart_event.event |= EPOLLET;

	event_instance.addEvent( uart_event );
	
	while ( 1 ) {
		event_instance.dispatcher();
	}

	ranging_sensor.release();

	return 0;
}
