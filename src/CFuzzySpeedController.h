#ifndef CFUZZYSPEEDCONTROLLER_H
#define CFUZZYSPEEDCONTROLLER_H

#include "ros/ros.h"
#include "fl/Headers.h"

class CFuzzySpeedController{

public:
	CFuzzySpeedController(const char* name = "Fuzzy Speed Controller", const char* description = "Speed controller");
	~CFuzzySpeedController();

	void getTarget(double* distanceTarget, double* angleTarget);
	void setTarget(const double& distanceTarget, const double& angleTarget);

	void getSystemInput(const double& distance, const double& angle, double* linearVelocity, double* angularVelocity);

private:
	double distanceTarget;
	double angleTarget;

	std::string name;
	std::string description;
	fl::Engine* engine;
	fl::InputVariable* distanceError;
	fl::InputVariable* angleError;
	fl::OutputVariable* linearVelocity;
	fl::OutputVariable* angularVelocity;
	fl::RuleBlock* ruleBlock;
};

#endif