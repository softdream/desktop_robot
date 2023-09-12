#include <wiringPi.h>
#include <iostream>

#define RA 25
#define RB 29

void callback()
{
	std::cout<<"callback ..."<<std::endl;
}

int main()
{
	wiringPiSetup();

	pinMode( RA, INPUT );

	pullUpDnControl( RA, PUD_UP );

	delay(100);

	wiringPiISR( RA, INT_EDGE_BOTH, callback );

	while ( 1 ) {
	
	}

	return 0;
}
