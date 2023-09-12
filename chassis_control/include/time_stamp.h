#ifndef __TIME_STAMP_H
#define __TIME_STAMP_H

#include <sys/time.h>
#include <time.h>

namespace stamp
{

class TimeStamp
{
public:
	static uint64_t getCurrentTimeMs()
	{
		struct timeval tv;
		gettimeofday( &tv, NULL );
		
		return tv.tv_sec * 1000 + tv.tv_usec / 1000;
	}

	static uint64_t getCurrentTimeUs()
	{
		struct timeval tv;
		gettimeofday( &tv, NULL );

		return tv.tv_sec * 1000000 + tv.tv_usec;
	}
};

}

#endif
