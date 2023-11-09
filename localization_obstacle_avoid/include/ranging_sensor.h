#ifndef __RANGING_SENSOR_H
#define __RANGING_SENSOR_H

#include "uart.h"

namespace sensor
{

class RangingSensor
{
public:
	RangingSensor()
	{
	
	}

	~RangingSensor()
	{
	
	}

	bool init()	
	{
		// 1. instance of the Uart
		uart_ptr_ = new uart::Uart();

		// 2. 
		if ( !setTimeInterval() ) return false;

		// 3. 
		if ( !setMeasuringRange() ) return false;

		return true;
	}

	template<typename T>
	const T getMeasuredVal()
	{
		int ret = uart_ptr_->read( recv_buffer_, 7 );
		if ( ret != 7 || recv_buffer_[0] != 0x01 ) return 0.0;

		int dist = recv_buffer_[3] * 256 + recv_buffer_[4]; // mm

		return ( static_cast<T>( dist ) * 0.001 ); // mm -> m
	}

	void release()
	{
		uart_ptr_->close();
		if ( uart_ptr_ != nullptr ) delete uart_ptr_;
	}

	const int getFd() const
	{
		return uart_ptr_->getFd();
	}

private:
	bool offsetCalibration()
	{
		return ( ( uart_ptr_->write( sys_cmd_[0], 8 ) == 8 ) ? true : false ); 
	}

	bool xtalkCalibraion()
	{
		return ( ( uart_ptr_->write( sys_cmd_[1], 8 ) == 8 ) ? true : false );
	}

	bool calibrationEnable()
	{
		return ( ( uart_ptr_->write( sys_cmd_[2], 8 ) == 8 ) ? true : false );
	}

	bool moduleReset()
	{
		return ( ( uart_ptr_->write( sys_cmd_[3], 8 ) == 8 ) ? true : false );
	}

	bool setTimeInterval()
	{
		return ( ( uart_ptr_->write( time_interval_cmd_, 8 ) == 8 ) ? true : false );	
	}

	bool setMeasuringRange()
	{
		return ( ( uart_ptr_->write( measure_range_cmd_, 8 ) == 8 ) ? true : false );
	}

private:
	constexpr static char sys_cmd_[4][8] = { { 0x01, 0x06, 0x00, 0x20, 0x00, 0x8C, 0xA5, 0x89 }, // offset distance 140mm
       						 { 0x01, 0x06, 0x00, 0x21, 0x00, 0x64, 0xD8, 0x2B }, // xtalk distance 100mm
						 { 0x01, 0x06, 0x00, 0x06, 0x00, 0x01, 0x0B, 0xA8 }, // load the calibration
						 { 0x01, 0x06, 0x00, 0x01, 0x10, 0x00, 0xCA, 0xD5 } }; // reset the module

	constexpr static char measure_range_cmd_[8] = { 0x01, 0x06, 0x00, 0x04, 0x00, 0x00, 0x0B, 0xC8 }; // measuring range [ 0 ~ 1300mm]

	constexpr static char time_interval_cmd_[8] = { 0x01, 0x06, 0x00, 0x05, 0x00, 0x64, 0x20, 0x98 }; // time interval = 100ms

private:
	uart::Uart* uart_ptr_ = nullptr;

	char recv_buffer_[7] = {0};
};

}

#endif
