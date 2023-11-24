#ifndef __TASK_PLANNER_H
#define __TASK_PLANNER_H



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
        value_type stamina_value = 0;
        value_type mood_value = 0;
        value_type entertainment_value = 0;
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
	
	}

	~TaskPlanner()
	{
	
	}


	// ------------------- 1. Stamina Value ------------------ //
	void staminaValueIncrease( const value_type delta = 1 )
	{
		attr_.stamina_value += delta;
		if ( attr_.stamina_value >= max_stamina_value_ ) attr_.stamina_value = max_stamina_value_;
	}

	void staminavalueDecrease( const value_type delta = 1 )
	{
		attr_.stamina_value -= delta;
		if ( attr_.stamina_value <= 0 ) attr_.stamina_value = 0;
	}

	// ------------------- 2. Mood Value ----------------------//
	void moodValueIncrease( const value_type delta = 1 )
	{
		attr_.mood_value += delta;
		if ( attr_.mood_value >= max_mood_value_ ) attr_.mood_value = max_mood_value_;
	}

	void moodValueDecrease( const value_type delta = 1 )
	{
		attr_.mood_value -= delta;
		if ( attr_.mood_value <= 0 ) attr_.mood_value = 0;
	}

	// --------------- 3. Entertainment Value  ----------------//
	void entertainmentValueIncrease( const value_type delta = 1 )
	{
		entertainment_value += delta;
		if ( attr_.entertainment_value >= max_entertainment_value_ ) entertainment_value = max_entertainment_value_;
	}

	void entertainmentValueDecrease( const value_type delta = 1 )
	{
		entertainment_value -= delta;
		if ( attr_.entertainment_value <= 0 ) entertainment_value = 0;
	}

	// ------------------------------------------------------- //
	void cacuNextStatus()
	{
		if ( attr_.error_value > error_value_thresh_ ) status_ = RelocalizationState;
		else if ( attr_.is_voltage_slow ) status_ = ChargingState;
		else if ( attr_.is_posture_abnormal ) status_ = PostureAbnormalState;	
		else if ( attr_.detected_obstacle ) status_ = SawObstacleState;
		else if ( attr_.detected_clif ) status_ = SawClifState;

		else {
		
		}
	}	

	const RobotStatus& getCurrentStatus() const
	{
		return status_;
	}	

public:
	// robot attributes
	RobotAttribution<value_type> attr_;

	// current status
	RobotStatus status_ = None;

	// parameters
	constexpr value_type error_value_thresh_ = 100; // relocalization threshold
	constexpr value_type max_stamina_value_ = 100;

	constexpr value_type max_mood_value_ = 100;
	constexpr value_type max_entertainment_value_ = 100;
};


}

#endif
