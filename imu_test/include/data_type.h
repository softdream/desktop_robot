#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

#include <stdint.h>

namespace sensor
{

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

}

#endif
