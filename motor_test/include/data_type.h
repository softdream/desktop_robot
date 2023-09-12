#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

#include <stdint.h>

namespace sensor
{

// -------------------------------- 1. IMU ------------------------------------ //
template<typename T>
struct ImuData_
{
	using value_type = T;

	ImuData_()
	{
	
	}

	ImuData_( const value_type acc_x_, const value_type acc_y_, const value_type acc_z_, const value_type gyro_x_, const value_type gyro_y_, const value_type gyro_z_ )
		: acc_x( acc_x_ ), acc_y( acc_y_ ), acc_z( acc_z_ ), gyro_x( gyro_x_ ), gyro_y( gyro_y_ ), gyro_z( gyro_z_ )
	{
	
	}

	value_type acc_x = 0;
	value_type acc_y = 0;
	value_type acc_z = 0;

	value_type gyro_x = 0;
	value_type gyro_y = 0;
	value_type gyro_z = 9;
};

template<typename T>
using ImuData = ImuData_<T>;

using ImuDataI = ImuData<int>;
using ImuDataF = ImuData<float>;
using ImuDataD = ImuData<double>;

// -------------------------------- 2. MOTOR ------------------------------------ //
template<typename T>
struct MotorData_
{
	using value_type = T;
	
	MotorData_()
	{
	
	}

	MotorData_( const value_type l_rpm_,
		    const value_type r_rpm_,
		    const value_type velocity_,
		    const value_type delta_s_,
		    const value_type delta_angle_ ) :
			l_rpm( l_rpm_ ),
			r_rpm( r_rpm_ ),
			velocity( velocity_ ),
			delta_s( delta_s_ ),
			delta_angle( delta_angle_ )
	{
	
	}

	value_type l_rpm = 0;
	value_type r_rpm = 0;
	value_type velocity = 0;
	value_type delta_s = 0;
	value_type delta_angle = 0;
};

template<typename T>
using MotorData = MotorData_<T>;

using MotorDataI = MotorData<int>;
using MotorDataF = MotorData<float>;
using MotorDataD = MotorData<double>;

}

#endif