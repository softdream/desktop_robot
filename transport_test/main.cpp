#include "motor.h"
#include "event_manage.h"
#include "timer.h"
#include "imu_driver.h"
#include "data_transport.h"
#include "ekf_fusion.h"

#include <unistd.h>

#include <thread>

// -------------------------------- GLOBAL DATA ------------------------------ //
event::EpollEvent event_instance;
timer::Timer timer_instance;
motor::Motor<float> motor_instance;
imu::MPU6050<float> mpu_6050;

transport::Receiver recver(2333);

sensor::MotorDataF motor_data;
sensor::ImuDataF imu_data;
// --------------------------------------------------------------------------- //

// udp receive callback function
void recverCallback( int fd, void* arg )
{
	std::cout<<"recver callback ..."<<std::endl;

	std::string str;
	recver.receive( str );
	std::cout<<"recv str = "<<str<<std::endl;
}

// timer callback function
void timerCallback( int fd, void* arg )
{
        std::cout<<"timer callback ..."<<std::endl;

	// 1. timer handle
        auto ret = timer_instance.handleRead();
	
	// 2. motor control
	//motor_instance.motorControl( motor_data );

}


void chassisControlThread()
{
	// 1. Init Motors & Encoders
	//motor_instance.initMotors();
        //motor_instance.initEncoders();

	// 2. Create a timer
	timer_instance.createTimer();

	// 3. Epoll : add a timer event
	event::Event event_timer( timer_instance.getTimerFd(), EPOLLIN, timerCallback, nullptr );
        event_instance.addEvent( event_timer );

	// 4. Epoll : add a udp event
	event::Event event_recver( recver.getSocketFd(), EPOLLIN, recverCallback, nullptr );
	event_instance.addEvent( event_recver );

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
	std::cout<<"------------------------ TRANSPORT TEST -------------------------"<<std::endl;
	//mpu_6050.init();
	//mpu_6050.calibration();

	std::thread t1( chassisControlThread );
	t1.join();

	return 0;
}
