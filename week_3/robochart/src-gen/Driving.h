#ifndef ROBOCALC_CONTROLLERS_DRIVING_H_
#define ROBOCALC_CONTROLLERS_DRIVING_H_

#include "TurtleBot.h"
#include "RoboCalcAPI/Controller.h"
#include "DataTypes.h"

#include "TurtleBot3FSMRC.h"

class Driving: public robocalc::Controller 
{
public:
	Driving(TurtleBot& _platform) : platform(&_platform){};
	Driving() : platform(nullptr){};
	
	~Driving() = default;
	
	void Execute()
	{
		turtleBot3FSMRC.execute();
	}
	
	struct Channels
	{
		Driving& instance;
		Channels(Driving& _instance) : instance(_instance) {}
		
	};
	
	Channels channels{*this};
	
	TurtleBot* platform;
	TurtleBot3FSMRC_StateMachine<Driving> turtleBot3FSMRC{*platform, *this, &turtleBot3FSMRC};
};

#endif
