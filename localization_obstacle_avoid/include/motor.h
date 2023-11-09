#ifndef __MOTOR_H
#define __MOTOR_H

#include <iostream>
#include <functional>
#include <wiringPi.h>

#include "pid.h"
#include "data_type.h"

// left motor
#define AIN1 4 
#define AIN2 5
#define PWMA 26

// right motor
#define BIN1 27
#define BIN2 28
#define PWMB 24

// left encoder
#define LA 22
#define LB 23
#define RA 25
#define RB 29

namespace motor
{

template<typename T>
class Motor
{
public:
	using value_type = T;

	void initMotors()
        {
                // 1. GPIO init
                pinMode( PWMA, PWM_OUTPUT );
                pinMode( AIN1, OUTPUT );
                pinMode( AIN2, OUTPUT );

                pinMode( PWMB, PWM_OUTPUT );
                pinMode( BIN1, OUTPUT );
                pinMode( BIN2, OUTPUT );

                digitalWrite( AIN1, LOW );
                digitalWrite( AIN2, LOW );
                digitalWrite( PWMA, LOW );

                digitalWrite( BIN1, LOW );
                digitalWrite( BIN2, LOW );
                digitalWrite( PWMB, LOW );
        }

        void initEncoders()
        {
                // 1. GPIO init
                pinMode( LA, INPUT );
                pinMode( LB, INPUT );
                pinMode( RA, INPUT );
                pinMode( RB, INPUT );

                // 2. GPIO configuration
                pullUpDnControl( LA, PUD_UP );
                pullUpDnControl( LB, PUD_UP );
                pullUpDnControl( RA, PUD_UP );
                pullUpDnControl( RB, PUD_UP );
		
		delay(100);

                // 3. set callback function
                wiringPiISR( LA, INT_EDGE_BOTH, leftEncoderCallback );
                wiringPiISR( RA, INT_EDGE_BOTH, rightEncoderCallback );
        }

	sensor::MotorData<value_type> motorControl()
	{
		// current rpm
		value_type l_rpm = 60 * static_cast<value_type>( l_enc_cnt ) / ( pulse_number_ * 0.05 );
		value_type r_rpm = -60 * static_cast<value_type>( r_enc_cnt ) / ( pulse_number_ * 0.05 );
	
		std::cout<<"l_rpm = "<<l_rpm<<", r_rpm = "<<r_rpm<<std::endl;

		value_type velocity = getVelocity( l_rpm, r_rpm );
		value_type delta_s = getDeltaS( l_enc_cnt, r_enc_cnt );
		value_type delta_angle = getDeltaAngle( l_enc_cnt, r_enc_cnt );
		
		// reset encoders' count
		l_enc_cnt = 0;
		r_enc_cnt = 0;
	
		// pid control
		value_type l_out = l_pid_->caculate( required_l_rpm_, l_rpm );
		value_type r_out = r_pid_->caculate( required_r_rpm_, r_rpm );
		std::cout<<"l_out = "<<l_out<<", r_out = "<<r_out<<std::endl;

		// set the motor pwm
		setPwm( static_cast<int>( l_out ), static_cast<int>( r_out ) );
		//setPwm( rpm2Pwm( l_out ), rpm2Pwm( r_out ) );

		// get the motor data
		return sensor::MotorData<value_type>( l_rpm, r_rpm, velocity, delta_s, delta_angle );
	}

	void motorControl( sensor::MotorData<value_type>& motor_data )
        {
                // current rpm
                value_type l_rpm = 60 * static_cast<value_type>( l_enc_cnt ) / ( pulse_number_ * 0.05 );
                value_type r_rpm = -60 * static_cast<value_type>( r_enc_cnt ) / ( pulse_number_ * 0.05 );

                //std::cout<<"l_rpm = "<<l_rpm<<", r_rpm = "<<r_rpm<<std::endl;

		motor_data.l_rpm = l_rpm;
		motor_data.r_rpm = r_rpm;
                motor_data.velocity = getVelocity( l_rpm, r_rpm );
                motor_data.delta_s = getDeltaS( l_enc_cnt, r_enc_cnt );
                motor_data.delta_angle = getDeltaAngle( l_enc_cnt, r_enc_cnt );

                // reset encoders' count
                l_enc_cnt = 0;
                r_enc_cnt = 0;

                // pid control
                value_type l_out = l_pid_->caculate( required_l_rpm_, l_rpm );
                value_type r_out = r_pid_->caculate( required_r_rpm_, r_rpm );
                //std::cout<<"l_out = "<<l_out<<", r_out = "<<r_out<<std::endl;

                // set the motor pwm
        	setPwm( rpm2Pwm( l_out ), rpm2Pwm( r_out ) );
	}

	template<typename U>
	static void cacuRPM( const U v, const U w )
	{
		// 1. convert m/s to m/min
		U v_mins = v * 60;

		// 2. convert rad/s to rad/min
		U w_mins = w * 60;

		// 3. cacu
		U w_vel = w_mins * base_width_;

		U x_rpm = v_mins / circumference_;
		U tan_rpm = w_vel / circumference_;

		// 4. get the required rpm
		required_l_rpm_ = static_cast<value_type>( x_rpm - tan_rpm * 0.5 );
		required_r_rpm_ = static_cast<value_type>( x_rpm + tan_rpm * 0.5 );
		
		std::cout<<"required left rpm = "<<required_l_rpm_<<std::endl;
		std::cout<<"required right rpm = "<<required_r_rpm_<<std::endl;
	}

public:
	Motor()
	{
		std::cout<<"motor start ..."<<std::endl;
		// wiringPi
		wiringPiSetup();
	
		//wiringPiSetupGpio();

		// PID 
		l_pid_ = new pid::PID<value_type>( 0.05, 800, -800, 1, 0.04, 0.01 );
		r_pid_ = new pid::PID<value_type>( 0.05, 800, -800, 1, 0.04, 0.01 );
	}

	~Motor()
	{
		if ( l_pid_ != nullptr ) delete l_pid_;
		if ( r_pid_ != nullptr ) delete r_pid_;
	
		digitalWrite( PWMA, LOW );
		digitalWrite( PWMB, LOW );
	}

private:
	const int rpm2Pwm( const value_type rpm )
	{
		return ( rpm / max_rpm_ ) * 1000;
	}

	static void leftEncoderCallback()
	{
		// 1. LA : high && LB : low
		if ( digitalRead( LA ) && !digitalRead( LB ) ) {
			l_enc_cnt ++;
		}
		// 2. LA : low && LB : high
		else if ( !digitalRead( LA ) && digitalRead( LB ) ) {
			l_enc_cnt ++;
		}
		// 3. other
		else {
			l_enc_cnt --;
		}
	}

	static void rightEncoderCallback()
	{
		// 1. RA : high && RB : low
                if ( digitalRead( RA ) && !digitalRead( RB ) ) {
                        r_enc_cnt ++;
                }
                // 2. RA : low && RB : high
                else if ( !digitalRead( RA ) && digitalRead( RB ) ) {
                        r_enc_cnt ++;
                }
                // 3. other
                else {
                        r_enc_cnt --;
                }
	}

	const value_type getVelocity( const value_type l_motor_rpm, const value_type r_motor_rpm )
	{
		value_type l_motor_rps = l_motor_rpm / 60; // rpm -> rps
		value_type r_motor_rps = r_motor_rpm / 60; // rpm -> rps

		value_type l_v = l_motor_rps * circumference_;
		value_type r_v = r_motor_rps * circumference_;

		return ( l_v + r_v ) * 0.5;
	}

	const value_type getDeltaS( const long l_cnt_c, const long r_cnt_c )
	{
		value_type l_s = ( static_cast<value_type>( l_cnt_c ) / pulse_number_ ) * circumference_;
		value_type r_s = -( static_cast<value_type>( r_cnt_c ) / pulse_number_ ) * circumference_;

		return ( l_s + r_s ) * 0.5;
	}

	const value_type getDeltaAngle( const long l_cnt_c, const long r_cnt_c )
	{
		value_type l_s = ( static_cast<value_type>( l_cnt_c ) / pulse_number_ ) * circumference_;
                value_type r_s = -( static_cast<value_type>( r_cnt_c ) / pulse_number_ ) * circumference_;

		return ( r_s - l_s ) / base_width_;
	}

	void setPwm( const int l_motor_pwm, const int r_motor_pwm )
        {
                // left motor
                if ( l_motor_pwm >= 0 ) {
                        digitalWrite( AIN1, HIGH );
                        digitalWrite( AIN2, LOW );
                        pwmWrite( PWMA, std::abs( l_motor_pwm ) );
                }
                else {
                        digitalWrite( AIN1, LOW );
                        digitalWrite( AIN2, HIGH );
                        pwmWrite( PWMA, std::abs( l_motor_pwm ) );
                }

                // right motor
                if ( r_motor_pwm >= 0 ) {
                        digitalWrite( BIN1, HIGH );
                        digitalWrite( BIN2, LOW );
                        pwmWrite( PWMB, std::abs( r_motor_pwm ) );
                }
                else {
                        digitalWrite( BIN1, LOW );
                        digitalWrite( BIN2, HIGH );
                        pwmWrite( PWMB, std::abs( r_motor_pwm ) );
                }
        }


private:
	pid::PID<value_type>* l_pid_ = nullptr;
	pid::PID<value_type>* r_pid_ = nullptr;

	static value_type required_l_rpm_;
	static value_type required_r_rpm_;

	static long l_enc_cnt;
	static long r_enc_cnt;

	constexpr static value_type circumference_ = 0.084823;
	constexpr static value_type pulse_number_ = 2100;
	constexpr static value_type base_width_ = 0.052;
	constexpr static value_type max_rpm_ = 50;
};

template<typename T>
T Motor<T>::required_l_rpm_ = 0;

template<typename T>
T Motor<T>::required_r_rpm_ = 0;

template<typename T>
long Motor<T>::l_enc_cnt = 0;

template<typename T>
long Motor<T>::r_enc_cnt = 0;


using MotorF = Motor<float>;
using MotorD = Motor<double>;
using MotorI = Motor<int>;

}

#endif
