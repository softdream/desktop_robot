#include "imu_driver.h"
#include <unistd.h>

int main()
{
	std::cout<<"--------------------- IMU TEST -------------------"<<std::endl;

	imu::MPU6050<float> mpu_6050;
	mpu_6050.init();

	//mpu_6050.calibration();
	mpu_6050.calibrateGryoZ();

	/*while ( 1 ) {
		sensor::ImuDataF imu_data;

		mpu_6050.getImuData( imu_data );	
		
		//std::cout<<"acc_x : "<<imu_data.acc_x<<", acc_y : "<<imu_data.acc_y<<", acc_z : "<<imu_data.acc_z<<std::endl;
		//std::cout<<"gyro_x : "<<imu_data.gyro_x<<", gyro_y : "<<imu_data.gyro_y<<", gyro_z : "<<imu_data.gyro_z<<std::endl;
		std::cout<<"gyro z = "<<imu_data.gyro_z<<std::endl;

		usleep( 100000 );
	}*/

	while ( 1 ) {
		float gyro_z = 0.0;

                mpu_6050.getGyroZ( gyro_z );

                std::cout<<"gyro z = "<<gyro_z<<std::endl;

                usleep( 100000 );
	}

	return 0;
}

