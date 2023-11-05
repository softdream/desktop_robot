#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>

namespace uart
{

class Uart
{
public:
	Uart()
	{
		init( "/dev/ttyAMA0" );
	}

	Uart( const std::string& device_path )
	{
		init( device_path );
	}

	~Uart()
	{
	
	}

	int read( char* buffer, int len ) 
	{
		return ::read( fd_, buffer, len );
	}

	void close()
	{
		::close( fd_ );
	}

	int write( const char* data, int len )
	{
		return ::write( fd_, data, len );
	}

	template<typename T>
	int write( const T& data )
	{
		return ::write( fd_, &data, sizeof(data) );
	}

	const int getFd() const
	{
		return fd_;
	}

private:
	bool init( const std::string& device_path )
	{
		fd_ = ::open( device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
		if ( fd_ < 0 ) {
			std::cerr<<"Error in Openning the TTY DEVICE : "<<fd_<<std::endl;
			return false;
		}

		std::cout<<"Open the TTY DEVICE : "<<fd_<<std::endl;

		struct termios serial_port_settings;
		::tcgetattr( fd_, &serial_port_settings ); // get the current attributes of the serial port
		::cfsetispeed( &serial_port_settings, B115200 ); // set Read Speed as 115200 
		::cfsetospeed( &serial_port_settings, B115200 ); // set Write Speed as 115200

		serial_port_settings.c_cflag &= ~PARENB; // Disable the Parity Enable bit(PARENB)
		serial_port_settings.c_cflag &= ~CSTOPB; // Stop bit
		serial_port_settings.c_cflag &= ~CSIZE; // Clear the mask for setting the data size
		serial_port_settings.c_cflag |= CS8; // Set the data bits = 8
		serial_port_settings.c_cflag &= ~CRTSCTS; // No Hardware flow Control
		serial_port_settings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
		serial_port_settings.c_iflag &= ~( IXON | IXOFF | IXANY ); // Disable XON/XOFF flow control both i/p and o/p
		serial_port_settings.c_lflag &= ~( ICANON | ECHO | ISIG ); // Non Cannonical mode
		serial_port_settings.c_oflag &= ~OPOST; // No Output Processing raw format output
		serial_port_settings.c_cc[VMIN] = 0; // read at least 10 characters
		serial_port_settings.c_cc[VTIME] = 1; // wait Indefinetly

		if ( ( ::tcsetattr( fd_, TCSANOW, &serial_port_settings ) ) != 0 ) {
			std::cerr<<"Error in Setting attributes "<<std::endl;
			return false;
		}

		std::cout<<"BoudRate : 115200, StopBit : 1, Parity : none, Hardware Flow Control : none ."<<std::endl;

		::tcflush( fd_, TCIFLUSH );

		return true;
	}

private:
	int fd_ = -1;
};

}

#endif
