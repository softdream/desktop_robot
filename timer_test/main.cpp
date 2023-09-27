#include "timer.h"
#include "event_manage.h"
#include "time_stamp.h"

#include <unistd.h>

timer::Timer timer_instance;

void timerCallback( int fd, void* arg )
{
	std::cout<<"timer callback ..."<<std::endl;

	auto ret = timer_instance.handleRead();

	auto stamp = stamp::TimeStamp::getCurrentTimeMs();
	std::cout<<stamp<<std::endl;

	usleep(80000);
}

int main()
{
	std::cerr<<"---------------- TIMER TEST ------------------"<<std::endl;

	timer_instance.createTimer();
	timer_instance.setTimer( 1, 0, 0, 100000000 );


	event::EpollEvent event_instance;
	
	event::Event event( timer_instance.getTimerFd(), EPOLLIN, timerCallback, nullptr );

	event_instance.initEvent();
	event_instance.addEvent( event );

	while ( 1 ) {
		event_instance.dispatcher();
	}

	return 0;
}
