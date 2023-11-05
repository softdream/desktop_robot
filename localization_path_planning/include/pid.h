#ifndef __PID_H
#define __PID_H

namespace pid
{

template<typename T>
class PID
{
public:
	using value_type = T;

	PID()
	{
	
	}

	PID( const value_type dt, 
	     const value_type max,
	     const value_type min,
	     const value_type kp,
	     const value_type ki,
	     const value_type kd ) :
		dt_( dt ),
		max_( max ),
		min_( min ),
		kp_( kp ),
		ki_( ki ),
		kd_( kd )
	{
	
	}

	~PID()
	{
	
	}

	const value_type caculate( const value_type target, const value_type& current )
	{
		// 1. error
		value_type error = target - current;

		// 2. proportional
		value_type p_out = kp_ * error;

		// 3. integral
		integral_ += error;
		if ( target == 0 || error == 0 ) integral_ = 0;
		value_type i_out = ki_ * integral_;
	
		//std::cout<<"intergal = "<<integral_<<std::endl;

		// 4. derivative
		value_type deriv = error - pre_error_;
		value_type d_out = kd_ * deriv;

		// 5. total output
		value_type out = p_out + i_out + d_out;

		// limit
		if ( out > max_ ) out = max_;
		else if ( out < min_ ) out = min_;

		// update old values
		pre_error_ = error;

		return out;
	}
	
	const value_type caculate( const value_type error )
        {
                // 1. proportional
                value_type p_out = kp_ * error;

                // 3. integral
                integral_ += error;
                if ( error == 0 ) integral_ = 0;
                value_type i_out = ki_ * integral_;

                // 4. derivative
                value_type deriv = error - pre_error_;
                value_type d_out = kd_ * deriv;

                // 5. total output
                value_type out = p_out + i_out + d_out;

                // limit
                if ( out > max_ ) out = max_;
                else if ( out < min_ ) out = min_;

                // update old values
                pre_error_ = error;

                return out;
        }

private:
	value_type dt_ = 0.05; // 50ms
	value_type max_ = 1000;
	value_type min_ = -1000;
	value_type kp_ = 10;
	value_type ki_ = 0;
	value_type kd_ = 1.1;
	value_type pre_error_ = 0;
	value_type integral_ = 0;
};

}

#endif
