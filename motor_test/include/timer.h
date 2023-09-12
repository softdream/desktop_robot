#ifndef __TIMER_H
#define __TIMER_H

#include <iostream>
#include <sys/timerfd.h>
#include <unistd.h>
#include <sys/time.h>

namespace timer
{

class Timer
{
public:
	Timer()
	{
	
	}

	~Timer()
	{
	
	}

	bool createTimer()
	{
		timer_fd_ = timerfd_create( CLOCK_MONOTONIC, 0 );
		if ( timer_fd_ < 0 ) {
			std::cerr<<"Timer create failed !"<<std::endl;
			return false;
		}

		return true;
	}

	bool setTimer( const int initial_time_sec, const long initial_time_nsec, const int interval_time, const long interval_time_nsec )
	{
		struct itimerspec itimer;
		itimer.it_value.tv_sec = initial_time_sec; // initial expiration
		itimer.it_value.tv_nsec = initial_time_nsec;

		itimer.it_interval.tv_sec = interval_time; // interval for periodic timer
		itimer.it_interval.tv_nsec = interval_time_nsec;

		int ret = timerfd_settime( timer_fd_, 0, &itimer, NULL );
		if ( ret < 0 ) {
			std::cerr<<"Timer Setting Failed !"<<std::endl;
			return false;
		}

		return true;
	}
	
	void closeTimer()
	{
		close( timer_fd_ );
	}

	const uint64_t handleRead()
	{
		uint64_t timerfd_read;
		int ret = read( timer_fd_, &timerfd_read, sizeof( uint64_t ) );
		if ( ret != sizeof( uint64_t ) ) {
			std::cerr<<"Failed to Read !"<<std::endl;
			return 0;
		}

		return timerfd_read;
	}

	const int getTimerFd() const
        {
                return timer_fd_;
        }


private:
	int timer_fd_ = -1;

};

};

#endif
