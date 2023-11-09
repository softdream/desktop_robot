#ifndef __INFRARED_SENSOR_H
#define __INFRARED_SENSOR_H

#include <iostream>
#include <wiringPi.h>

#define INFRARED_PIN 20

namespace infrared
{

class InfraredSensor
{
public:
	InfraredSensor()
	{
		init();	
	}

	~InfraredSensor()
	{
	
	}

	int getPinValue()
	{
		return digitalRead( INFRARED_PIN );	
	}	

private:
	void init()
	{
		// 1. init the wiringpi
                wiringPiSetup();

		// 2. init gpio
		pinMode( INFRARED_PIN, INPUT );
	}
};	

}

#endif
