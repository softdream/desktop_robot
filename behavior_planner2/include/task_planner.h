#ifndef __TASK_PLANNER_H
#define __TASK_PLANNER_H



namespace planning
{

enum RobotStatus
{
	HangOutState,
	RestState,
	RelocalizationState,
	AbnormalState,
	ChargingState,
	EntertainmentState,
};


class TaskPlanner
{
public:
	TaskPlanner()
	{
	
	}

	~TaskPlanner()
	{
	
	}


	void staminaValueDecrease()
	{
		stamina_value_ -= 10;
		if ( stamina_value_ < 0 ) stamina_value_ = 0;
	}

	void staminavalueIncrease()
	{
		stamina_value_ += 1;
		if ( stamina_value_ > 100 ) stamina_value_ = 100;
	}

	void errorValueIncrease()
	{
		error_value_ += 40;
		if ( error_value_ > 100 ) error_value_ = 100;
	}

	void errorValueDecrease()
	{
		error_value_ = 0;
	}



	const RobotStatus cacuNextStatus()
	{
		if ( error_value_ >= 100 ) {
			return RelocalizationState;
		}
		else {
			if ( stamina_value_ >= 10 ) {
                	        return HangOutState;
                	}
	                else if ( stamina_value_ <= 0 ) {
        	                return RestState;
                	}
		}
	}		

private:
	// robot attributes
	static int stamina_value_;
	static int mood_value_;
	static int energy_value_;
	static int error_value_;
};

int TaskPlanner::stamina_value_ = 0;
int TaskPlanner::mood_value_ = 0;
int TaskPlanner::energy_value_ = 0;
int TaskPlanner::error_value_ = 0;

}

#endif
