#ifndef __TASK_PLANNER_H
#define __TASK_PLANNER_H

#include "MLP.h"
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
struct RobotAttribution_
{
	using value_type = T;

	// robot attributes
        value_type stamina_value = 10;
        value_type mood_value = 10;
        value_type entertainment_value = 10;
        value_type error_value = 0;
        value_type detected_person = false;
        value_type is_voltage_slow = false;
        value_type is_posture_abnormal = false;
        value_type detected_obstacle = false;
        value_type detected_clif = false;
};

template<typename T>
using RobotAttribution = RobotAttribution_<T>;

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

	TaskPlanner( const std::string& model_path )
	{
		initMLP( model_path );
		initRandomSystem();
	}

	void initRandomSystem()
	{
		gen_.seed( rd_() );
	}

	void initMLP( const std::string& model_path )
	{
		mlp_ptr_ = std::make_unique<MLP>( model_path );	
	}


	// ------------------- 1. Stamina Value ------------------ //
	void staminaValueIncrease( const value_type delta = 1 )
	{
		std::cout<<"stamina delta = "<<delta<<", val = "<<attr_.stamina_value<<std::endl;

		attr_.stamina_value += delta;
		if ( attr_.stamina_value >= max_stamina_value_ ) attr_.stamina_value = max_stamina_value_;
	}

	void staminavalueDecrease( const value_type delta = 1 )
	{
		std::cout<<"stamina delta = "<<delta<<", val = "<<attr_.stamina_value<<std::endl;

		attr_.stamina_value -= delta;
		if ( attr_.stamina_value <= 0 ) attr_.stamina_value = 0;
	}

	// ------------------- 2. Mood Value ----------------------//
	void moodValueIncrease( const value_type delta = 1 )
	{
		std::cout<<"mood delta = "<<delta<<", val = "<<attr_.mood_value<<std::endl;

		attr_.mood_value += delta;
		if ( attr_.mood_value >= max_mood_value_ ) attr_.mood_value = max_mood_value_;
	}

	void moodValueDecrease( const value_type delta = 1 )
	{
		std::cout<<"mood delta = "<<delta<<", val = "<<attr_.mood_value<<std::endl;

		attr_.mood_value -= delta;
		if ( attr_.mood_value <= 1 ) attr_.mood_value = 1;
	}

	// --------------- 3. Entertainment Value  ----------------//
	void entertainmentValueIncrease( const value_type delta = 1 )
	{
		std::cout<<"entertainment delta = "<<delta<<", val = "<<attr_.entertainment_value<<std::endl;

		attr_.entertainment_value += delta;
		if ( attr_.entertainment_value >= max_entertainment_value_ ) attr_.entertainment_value = max_entertainment_value_;
	}

	void entertainmentValueDecrease( const value_type delta = 1 )
	{
		std::cout<<"entertainment delta = "<<delta<<", val = "<<attr_.entertainment_value<<std::endl;

		attr_.entertainment_value -= delta;
		if ( attr_.entertainment_value <= 1 ) attr_.entertainment_value = 1;
	}

	// -------------------- 4. Error Value ------------------- //
	void errorValueIncrease( const value_type delta = 1 )
	{
		std::cout<<"error val = "<<attr_.error_value<<std::endl;

		attr_.error_value += delta;
		if ( attr_.error_value >= error_value_thresh_ ) attr_.error_value = error_value_thresh_;
	}

	void errorValueReset()
	{
		attr_.error_value = 0;
	}
	
	// ---------------- 5. Detected a obstacle --------------- //
	void setDetectedObstacle()
	{
		attr_.detected_obstacle = true;
	}

	void resetDetectedObstacle()
	{
		attr_.detected_obstacle = false;
	}

	// ------------------------------------------------------- //
	const RobotStatus cacuNextStatus()
	{
		printAttribution();

		// current status
        	RobotStatus status_ = None;

		if ( attr_.error_value >= error_value_thresh_ ) status_ = RelocalizationState;
		else if ( attr_.is_voltage_slow ) status_ = ChargingState;
		//else if ( attr_.is_posture_abnormal ) status_ = PostureAbnormalState;	
		else if ( attr_.detected_obstacle ) status_ = SawObstacleState;
		else if ( attr_.detected_clif ) status_ = SawClifState;

		else {
			status_ = mlpPredict();
		}
	
		return status_;
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

private:
	const RobotStatus mlpPredict()
	{
		std::vector<double> in_vec = { static_cast<double>( attr_.stamina_value ),
	       				       static_cast<double>( attr_.mood_value ),
					       static_cast<double>( attr_.entertainment_value )	};

		std::vector<double> guess;
		mlp_ptr_->GetOutput( in_vec, &guess );

		size_t class_id = -1;
		mlp_ptr_->GetOutputClass( guess, &class_id );

		if ( class_id == 0 ) {
			return HangOutState;
		} 
		else if ( class_id == 1 ) {
			return RestState;	
		}
		else if ( class_id == 2 ) {
			return EntertainmentState;
		}
		else if ( class_id == 3 ) {
			return EnjoymentState;
		}
	}

	void printAttribution()
	{
		std::cout<<" ------------ ATTR -------------- "<<std::endl;
		std::cout<<"stamina_value : "<<attr_.stamina_value<<std::endl;
		std::cout<<"mood_value : "<<attr_.mood_value<<std::endl;
		std::cout<<"entertainment_value : "<<attr_.entertainment_value<<std::endl;
		std::cout<<"error_value : "<<attr_.error_value<<std::endl;
		std::cout<<"detected_person : "<<attr_.detected_person<<std::endl;
		std::cout<<"is_voltage_slow : "<<attr_.is_voltage_slow<<std::endl;
		std::cout<<"is_posture_abnormal : "<<attr_.is_posture_abnormal<<std::endl;
		std::cout<<"detected_obstacle : "<<attr_.detected_obstacle<<std::endl;
		std::cout<<"detected_clif : "<<attr_.detected_clif<<std::endl;
		std::cout<<" -------------------------------- "<<std::endl;
	}

private:
	// robot attributes
	RobotAttribution<value_type> attr_;

	// parameters
	static constexpr value_type error_value_thresh_ = 100; // relocalization threshold
	static constexpr value_type max_stamina_value_ = 100;

	static constexpr value_type max_mood_value_ = 100;
	static constexpr value_type max_entertainment_value_ = 100;


	// MLP
	std::unique_ptr<MLP> mlp_ptr_;

	// random system
	std::random_device rd_;
	std::mt19937 gen_;
};


}

#endif
