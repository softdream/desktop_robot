#ifndef __PATH_TRACKING_H
#define __PATH_TRACKING_H

#include "pid.h"

namespace planning
{

template<typename T>
class Tracking
{
public:
	using value_type = T;

	Tracking()
	{
		yaw_pid_ = new pid::PID<value_type>( 0.2, 1.5, -1.5, 0.8, 0.001, 0.01 );
	}

	~Tracking()
	{
		delete yaw_pid_;
	}

	
	const std::pair<value_type, value_type>
	cacuControlVector( const value_type target_yaw, const value_type curr_yaw )
	{
		value_type error = target_yaw - curr_yaw;

		if ( std::abs( error ) >= ( M_PI * 0.15 ) ) {
			auto w = yawPidProcess( error );
			return { 0.0, w };
		}
		else {
			auto w = yawPidProcess( error );
			return { 0.05, w };
		}

		return { 0.0, 0.0 };
	}

	const std::pair<value_type, value_type>
	cacuControlVectorForRotation( const value_type target_yaw, const value_type curr_yaw )
	{
		value_type error = target_yaw - curr_yaw;

		auto w = yawPidProcess( error );

		return { 0.0, w };
	}

private:
	const value_type yawPidProcess( const value_type error )
	{
		return yaw_pid_->caculate( error );
	}

private:
	pid::PID<value_type>* yaw_pid_ = nullptr;	
};

}

#endif
