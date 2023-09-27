#ifndef __EVENT_MANAGE_H
#define __EVENT_MANAGE_H

#include <iostream>
#include <sys/epoll.h>

#include <functional>
#include <map>

namespace event
{
	
using Callback = std::function<void( int, void* )>;

struct Event_
{
	Event_()
	{
	
	}

	Event_( const int fd_, const short event_, const Callback callback_, void* arg_ ) : 
		fd( fd_ ), event( event_ ), callback( callback_ ), arg( arg_ )
	{
	
	}

	~Event_()
	{
	
	}

	int fd = -1;
	short event = -1;
	Callback callback = nullptr;
	void* arg = nullptr;
};

using Event = Event_;

class EpollEvent
{
public:
	EpollEvent()
	{
		if ( initEvent() ) {
			std::cout<<"Create a Epoll Event Management System !"<<std::endl;
		}
	}

	~EpollEvent()
	{
	
	}

	bool initEvent()
	{
		epoll_fd_ = epoll_create( epoll_create_size_ );
		if ( epoll_fd_ <= 0 ) {
			std::cerr<<"Can not create Epoll !"<<std::endl;
			return false;
		}
		
		return true;
	}

	bool addEvent( const Event& event )
	{
		struct epoll_event e_event;
		e_event.data.fd = event.fd;
		e_event.events = event.event;

		int ret = epoll_ctl( epoll_fd_, EPOLL_CTL_ADD, event.fd, &e_event );
		if ( ret < 0 ) {
			std::cerr<<"epoll_ctl error !"<<ret<<std::endl;
			return false;
		}

		events_map_[event.fd] = event;

		return true;
	}

	bool delEvent( const Event& event )
	{
		struct epoll_event e_event;
		e_event.data.fd = event.fd;
		e_event.events = event.event;

		int ret = epoll_ctl( epoll_fd_, EPOLL_CTL_DEL, event.fd, &e_event );
		if ( ret < 0 ) {
                        std::cerr<<"epoll_ctl error : "<<ret<<std::endl;
                        return false;
                }
	
		events_map_.erase( event.fd );

		return true;
	}

	bool dispatcher()
	{
		struct epoll_event e_events[epoll_create_size_];

		int n_events = epoll_wait( epoll_fd_, e_events, epoll_create_size_, -1 );

		if ( n_events <= 0 ) {
			std::cerr<<"epoll_wait error : "<<n_events<<std::endl;
			return false;
		}

		// handle the events
		for ( int i = 0; i < n_events; i ++ ) {
			int fd = e_events[i].data.fd;
			Event event = events_map_[fd];

			if ( event.callback ) {
				event.callback( fd, event.arg );
			}
		}

		return true;
	}

private:

	constexpr static int epoll_create_size_ = 16;

	int epoll_fd_ = -1;
	std::map<int, Event> events_map_;

};

}

#endif
