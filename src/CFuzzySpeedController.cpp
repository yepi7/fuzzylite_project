#include "CFuzzySpeedController.h"

CFuzzySpeedController::CFuzzySpeedController(const char* name, const char* description){
	engine = new fl::Engine;
	engine->setName("SpeedController");
	engine->setDescription("");

	//distances in meters
	distanceError = new fl::InputVariable;
	distanceError->setName("distanceError");
	distanceError->setDescription("");
	distanceError->setEnabled(true);
	distanceError->setRange(0.0, std::numeric_limits<double>::infinity());
	distanceError->setLockValueInRange(false);
	distanceError->addTerm(new fl::Triangle("VNear", 0.0, 0.0, 0.01));
	distanceError->addTerm(new fl::Triangle("Near", 0.0, 0.01, 1));
	distanceError->addTerm(new fl::Triangle("Far", 0.01, 1, 3));
	distanceError->addTerm(new fl::Trapezoid("VFar", 1, 3, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(distanceError);

	//radians
	angleError = new fl::InputVariable;
	angleError->setName("angleError");
	angleError->setDescription("");
	angleError->setEnabled(true);
	angleError->setRange(-M_PI, M_PI);
	angleError->setLockValueInRange(false);
	angleError->addTerm(new fl::Triangle("VLeft", -std::numeric_limits<double>::infinity(), -M_PI/4, -M_PI/90));
	angleError->addTerm(new fl::Triangle("Left", -M_PI/4, -M_PI/90, 0));
	angleError->addTerm(new fl::Triangle("Straight", -M_PI/90, 0, M_PI/90));
	angleError->addTerm(new fl::Triangle("Right", 0, M_PI/90, M_PI/4));
	angleError->addTerm(new fl::Triangle("VRight", M_PI/90, M_PI/4, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(angleError);


	//linear speed in mm/s
	linearVelocity = new fl::OutputVariable;
	linearVelocity->setName("linearVelocity");
	linearVelocity->setDescription("");
	linearVelocity->setEnabled(true);
	linearVelocity->setRange(0.0, 1.0);
	linearVelocity->setLockValueInRange(false);
	linearVelocity->setAggregation(new fl::AlgebraicSum);
	linearVelocity->setDefuzzifier(new fl::Centroid(100));
	linearVelocity->setDefaultValue(0.0);
	linearVelocity->setLockPreviousValue(false);
	linearVelocity->addTerm(new fl::Triangle("ZeroLin", 0.0, 0.0, 0.25)); // Va de 0 a 0.25
	linearVelocity->addTerm(new fl::Triangle("SlowForward", 0.0, 0.25, 0.5));
	linearVelocity->addTerm(new fl::Triangle("MediumForward", 0.25, 0.5, 0.75));
	linearVelocity->addTerm(new fl::Triangle("FastForward", 0.5, 0.75, 1.0));
	engine->addOutputVariable(linearVelocity);

	
	angularVelocity = new fl::OutputVariable;
	angularVelocity->setName("angularVelocity");
	angularVelocity->setDescription("");
	angularVelocity->setEnabled(true);
	angularVelocity->setRange(-1.000, 1.000);
	angularVelocity->setLockValueInRange(false);
	angularVelocity->setAggregation(new fl::AlgebraicSum);
	angularVelocity->setDefuzzifier(new fl::Centroid(100));
	angularVelocity->setDefaultValue(0.0);
	angularVelocity->setLockPreviousValue(false);
	angularVelocity->addTerm(new fl::Triangle("FastRight", -1.000, -0.750, -0.500));
	angularVelocity->addTerm(new fl::Triangle("SlowRight", -0.750, -0.500, -0.250));
	angularVelocity->addTerm(new fl::Triangle("ZeroAng", -0.250, 0.000, 0.250));
	angularVelocity->addTerm(new fl::Triangle("SlowLeft", 0.250, 0.500, 0.750));
	angularVelocity->addTerm(new fl::Triangle("FastLeft", 0.500, 0.750, 1.000));
	engine->addOutputVariable(angularVelocity);

	ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setImplication(new fl::AlgebraicProduct);
	ruleBlock->setActivation(new fl::General);

	ruleBlock->addRule(fl::Rule::parse("if angleError is VLeft then angularVelocity is FastRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Left then angularVelocity is SlowRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Straight then angularVelocity is ZeroAng", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Right then angularVelocity is SlowLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is VRight then angularVelocity is FastLeft", engine));

	ruleBlock->addRule(fl::Rule::parse("if distanceError is VNear then linearVelocity is FastForward", engine));
	ruleBlock->addRule(fl::Rule::parse("if distanceError is Near then linearVelocity is MediumForward", engine));
	ruleBlock->addRule(fl::Rule::parse("if distanceError is Far then linearVelocity is SlowForward", engine));
	ruleBlock->addRule(fl::Rule::parse("if distanceError is VFar then linearVelocity is ZeroLin", engine));

	engine->addRuleBlock(ruleBlock);
	fl::fuzzylite::setDebugging(false);
	ROS_INFO("%s has been instantiated...", name);
}

CFuzzySpeedController::~CFuzzySpeedController(){
	delete engine;
}	

void CFuzzySpeedController::getTarget(double* distanceTarget, double* angleTarget){
	*distanceTarget = this->distanceTarget;
	*angleTarget = this->angleTarget;
}

void CFuzzySpeedController::setTarget(const double& distanceTarget, const double& angleTarget){
	this->distanceTarget = distanceTarget;
	this->angleTarget = angleTarget;
}

void CFuzzySpeedController::getSystemInput(const double& distance, const double& angle, double* linearVelocity, double* angularVelocity){

	this->distanceError->setValue(distance);
	this->angleError->setValue(angle);

	engine->process();

	*linearVelocity = this->linearVelocity->getValue();
	*angularVelocity = this->angularVelocity->getValue();
}