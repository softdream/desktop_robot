#include "motor.h"
#include "event_manage.h"
#include "timer.h"
#include "imu_driver.h"

#include <unistd.h>

#include <thread>

// -------------------------------- GLOBAL DATA ------------------------------ //
event::EpollEvent event_instance;
timer::Timer timer_instance;
motor::Motor<float> motor_instance;
imu::MPU6050<float> mpu_6050;

sensor::MotorDataF motor_data;
sensor::ImuDataF imu_data;
// --------------------------------------------------------------------------- //

// timer callback function
void timerCallback( int fd, void* arg )
{
        std::cout<<"timer callback ..."<<std::endl;

	// 1. timer handle
        auto ret = timer_instance.handleRead();
	
	// 2. motor control
	motor_instance.motorControl( motor_data );

	// 3. get imu data
	mpu_6050.getImuData( imu_data );
}


void chassisControlThread()
{
	// 1. Init Motors & Encoders
	motor_instance.initMotors();
        motor_instance.initEncoders();

	// 2. Create a timer
	timer_instance.createTimer();

	// 3. Epoll : add a timer event
	event::Event event( timer_instance.getTimerFd(), EPOLLIN, timerCallback, nullptr );
        event_instance.addEvent( event );

	// 4. start the timer
	timer_instance.setTimer( 1, 0, 0, 50000000 ); // 50ms

	// 5. Epoll wait
	while ( 1 ) {
                event_instance.dispatcher();
        }

	return;
}

int main()
{
	std::cout<<"------------------------ MOTOR TEST -------------------------"<<std::endl;
	mpu_6050.init();
	mpu_6050.calibration();

	return 0;
}
