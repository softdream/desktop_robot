#ifndef __IMU_DRIVER_H
#define __IMU_DRIVER_H

#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "data_type.h"

#define MPU6050_ADDRESS (0x68)
#define MPU6050_REG_PWR_MGMT_1 (0x6b)
#define MPU6050_REG_DATA_START (0x3b)
#define A_SCALE (16384.0)
#define ANG_SCALE (131.0)

namespace imu
{

class MPU6050
{
public:
	MPU6050() 
	{
	
	}

	~MPU6050()
	{
	
	}

	bool init()
	{
		// 1. init the wiringpi
		wiringPiSetup();

		// 2. intit the IIC 
		fd = wiringPiI2CSetup( MPU6050_ADDRESS );
		if ( fd < 0 ) {
			std::cerr<<"Cannot Open the IMU Device !"<<std::endl;
			return false;
		}
		
		// 3. start the MPU6050
		wiringPiI2CWriteReg8( fd, MPU6050_REG_PWR_MGMT_1, 0 );

		return true;
	}

	template<typename T>
	void getImuData( sensor::ImuData<T>& imu_data )
	{
		// 1. 
		uint8_t msb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START );
		uint8_t lsb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 1 );
		short accel_x = msb << 8 | lsb;

		// 2.
		msb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 2 );
		lsb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 3 );
		short accel_y = msb << 8 | lsb;

		// 3.
                msb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 4 );
                lsb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 5 );
                short accel_z = msb << 8 | lsb;

		// 4.
                msb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 8 );
                lsb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 9 );
                short gyro_x = msb << 8 | lsb;

		// 5.
                msb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 10 );
                lsb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 11 );
                short gyro_y = msb << 8 | lsb;

		// 6.
                msb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 12 );
                lsb = wiringPiI2CReadReg8( fd, MPU6050_REG_DATA_START + 13 );
                short gyro_z = msb << 8 | lsb;

		imu_data.acc_x = static_cast<T>( accel_x ) / A_SCALE;
		imu_data.acc_y = static_cast<T>( accel_y ) / A_SCALE;
		imu_data.acc_z = static_cast<T>( accel_z ) / A_SCALE;

		imu_data.gyro_x = static_cast<T>( gyro_x ) / ANG_SCALE;
		imu_data.gyro_y = static_cast<T>( gyro_y ) / ANG_SCALE;
		imu_data.gyro_z = static_cast<T>( gyro_z ) / ANG_SCALE;
	}

private:
	int fd = -1;
};

}

#endif
