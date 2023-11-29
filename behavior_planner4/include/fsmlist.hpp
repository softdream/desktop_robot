#ifndef FSMLIST_HPP_INCLUDED
#define FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>

#include "robot_motion.h"

using fsm_list = tinyfsm::FsmList<RobotMotion>;

/** dispatch event to "RobotMotion" */
template<typename E>
void send_event(E const & event)
{
	fsm_list::template dispatch<E>(event);
}


#endif
