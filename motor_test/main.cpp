#include "motor.h"

#include <unistd.h>

int main()
{
	std::cout<<"------------------------ MOTOR TEST -------------------------"<<std::endl;
	
	motor::Motor<float> motor_instance;	

	motor_instance.initMotors();
	motor_instance.initEncoders();

	while ( 1 ) {
		motor_instance.motorControl();
		usleep( 100000 );
	}

	return 0;
}
