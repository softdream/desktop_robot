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

template<typename T>
class MPU6050
{
public:
	using value_type = T;

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

	void calibration()
	{
		std::cout<<"------------------- START IMU CALIBRATION -------------------"<<std::endl;
		int offset_cnt = 0;

		while ( offset_cnt < 1000 ) {
			sensor::ImuData<value_type> imu_data;
			getImuData( imu_data );

			acc_x_offset_ += imu_data.acc_x;
                	acc_y_offset_ += imu_data.acc_y;
	                acc_z_offset_ += imu_data.acc_z;

        	        gyro_x_offset_ += imu_data.gyro_x;
                	gyro_y_offset_ += imu_data.gyro_y;
	                gyro_z_offset_ += imu_data.gyro_z;

			offset_cnt ++;
		}	

		acc_x_offset_ = acc_x_offset_ / static_cast<value_type>( offset_cnt );
                acc_y_offset_ = acc_y_offset_ / static_cast<value_type>( offset_cnt );
                acc_z_offset_ = acc_z_offset_ / static_cast<value_type>( offset_cnt );

		gyro_x_offset_ = gyro_x_offset_ / static_cast<value_type>( offset_cnt );
                gyro_y_offset_ = gyro_y_offset_ / static_cast<value_type>( offset_cnt );
                gyro_z_offset_ = gyro_z_offset_ / static_cast<value_type>( offset_cnt );

                acc_z_offset_ -= 9.79362;
	
		std::cout<<"acc_x_offset_ = "<<acc_x_offset_<<std::endl;
		std::cout<<"acc_y_offset_ = "<<acc_y_offset_<<std::endl;
		std::cout<<"acc_y_offset_ = "<<acc_y_offset_<<std::endl;
		std::cout<<"gyro_x_offset_ = "<<gyro_x_offset_<<std::endl;
		std::cout<<"gyro_y_offset_ = "<<gyro_y_offset_<<std::endl;
		std::cout<<"gyro_z_offset_ = "<<gyro_z_offset_<<std::endl;

		std::cout<<"----------------------- END CALIBRATION --------------------"<<std::endl;
	}

	void getImuData( sensor::ImuData<value_type>& imu_data )
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

		imu_data.acc_x = static_cast<value_type>( accel_x ) / A_SCALE - acc_x_offset_;
		imu_data.acc_y = static_cast<value_type>( accel_y ) / A_SCALE - acc_y_offset_;
		imu_data.acc_z = static_cast<value_type>( accel_z ) / A_SCALE - acc_z_offset_;

		imu_data.gyro_x = static_cast<value_type>( gyro_x ) / ANG_SCALE - gyro_x_offset_;
		imu_data.gyro_y = static_cast<value_type>( gyro_y ) / ANG_SCALE - gyro_y_offset_;
		imu_data.gyro_z = static_cast<value_type>( gyro_z ) / ANG_SCALE - gyro_z_offset_;
	}

private:
	int fd = -1;

	// calibration
	value_type acc_x_offset_ = 0.0;
	value_type acc_y_offset_ = 0.0;
	value_type acc_z_offset_ = 0.0;

	value_type gyro_x_offset_ = 0.0;
	value_type gyro_y_offset_ = 0.0;
	value_type gyro_z_offset_ = 0.0;

	int offset_cnt = 0;
};

}

#endif
