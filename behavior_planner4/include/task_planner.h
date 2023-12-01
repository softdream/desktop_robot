#ifndef __TASK_PLANNER_H
#define __TASK_PLANNER_H

#include <memory>

#include <random>

namespace planning
{

enum RobotStatus
{
	None,
	HangOutState,
	RestState,
	EntertainmentState,
	EnjoymentState,

	RelocalizationState,
	PostureAbnormalState,
	TaskingState,
	SawObstacleState,
	SawClifState,
	ChargingState,

	ErrorState,
};

template<typename T>
struct InputValue_
{
	using value_type = T;

        value_type error_value = 0;
        bool detected_person = false;
        bool is_voltage_slow = false;
        bool is_posture_abnormal = false;
        bool detected_obstacle = false;
        bool detected_clif = false;
};

template<typename T>
using InputValue = InputValue_<T>;

template<typename T>
class TaskPlanner
{
public:
	using value_type = T;

	TaskPlanner()
	{
		initRandomSystem();
	}

	~TaskPlanner()
	{
	
	}


	void initRandomSystem()
	{
		gen_.seed( rd_() );
	}


	// -------------------- 1. Error Value ------------------- //
	void errorValueIncrease( const value_type delta = 1 )
	{
		std::cout<<"error val = "<<input_val_.error_value<<std::endl;

		input_val_.error_value += delta;
		if ( input_val_.error_value >= error_value_thresh_ ) input_val_.error_value = error_value_thresh_;
	}

	void errorValueReset()
	{
		input_val_.error_value = 0;
	}
	
	// ---------------- 2. Detected a obstacle --------------- //
	void setDetectedObstacle()
	{
		input_val_.detected_obstacle = true;
	}

	void resetDetectedObstacle()
	{
		input_val_.detected_obstacle = false;
	}

	// ------------------------------------------------------- //
	const RobotStatus cacuNextStatus()
	{
		// current status
        	RobotStatus status = None;

		if ( input_val_.error_value >= error_value_thresh_ ) status = RelocalizationState;
		else if ( input_val_.is_voltage_slow ) status = ChargingState;
		//else if ( input_val_.is_posture_abnormal ) status = PostureAbnormalState;	
		else if ( input_val_.detected_obstacle ) status = SawObstacleState;
		else if ( input_val_.detected_clif ) status = SawClifState;

		else {
			std::random_device rd;
                	std::mt19937 gen( rd() );

	                std::uniform_int_distribution<int> dist( 0, 3 );

			switch ( dist( gen ) ) {
				case 0 : { std::cout<<"HangOut"<<std::endl; status = HangOutState; break; }
				case 1 : { std::cout<<"Rest"<<std::endl; status = RestState; break; }
				case 2 : { std::cout<<"Entertainment"<<std::endl; status = EntertainmentState; break; }
				case 3 : { std::cout<<"Enjoyment"<<std::endl; status = EnjoymentState; break; }
				default : break;
			}
		}
	
		return status;
	}	

	const value_type randomIncrement()
        {
                std::uniform_real_distribution<value_type> val( 0.0, 1.0 );

                return val( gen_ );
        }

	const int randomTimeDuration()
	{
		std::random_device rd;
        	std::mt19937 gen( rd() );

        	std::uniform_int_distribution<int> dist( 120, 280 );

		return dist( gen );
	}

	const int getRandomValue( const int start, const int end )
        {
                std::random_device rd;
                std::mt19937 gen( rd() );

                std::uniform_int_distribution<int> dist( start, end );

                return dist( gen );
        }

private:
	// robot attributes
	InputValue<value_type> input_val_;

	// parameters
	static constexpr value_type error_value_thresh_ = 100; // relocalization threshold

	// random system
	std::random_device rd_;
	std::mt19937 gen_;
};


}

#endif
