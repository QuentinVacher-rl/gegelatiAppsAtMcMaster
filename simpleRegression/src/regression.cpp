#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "regression.h"


std::vector<std::reference_wrapper<const Data::DataHandler>> RegressionWrapper::getDataSources()
{
	auto result = std::vector<std::reference_wrapper<const Data::DataHandler>>();
	result.push_back(this->currentState);
	return result;
}

void RegressionWrapper::reset(size_t seed, Learn::LearningMode mode, uint16_t iterationNumber, uint64_t generationNumber)
{
	// Create seed from seed and mode
	
	size_t hash_seed = Data::Hash<size_t>()(seed) ^ Data::Hash<Learn::LearningMode>()(mode);
	if(mode == Learn::LearningMode::VALIDATION){
		hash_seed = 6416846135168433 + iterationNumber;
		
	}

	isValidation = (mode == Learn::LearningMode::VALIDATION);

	// Reset the RNG
	this->rng.setSeed(hash_seed);

	// Set initial state
	this->setState(this->rng.getDouble(-2, 2));
	this->nbActionsExecuted = 0;
	this->totalReward = 0.0;
}

void RegressionWrapper::setState(double newValue)
{

	this->currentState.setDataAt(typeid(double), 0, newValue);

}

double RegressionWrapper::getState() const
{
	return *this->currentState.getDataAt(typeid(const double), 0).getSharedPointer<const double>();
}

void RegressionWrapper::doActions(std::vector<double> actionsID)
{
	double value = std::sin(
		1 * std::pow(this->getState(), 2) -
		2 * this->getState() + 1
	);
	 value = std::sin(
		this->getState() * this->getState() + this->getState()
	);
	if(isValidation)std::cout<<this->getState()<<","<<value<<","<<actionsID[0]<<std::endl;

	double reward = - (std::pow(actionsID[0] - value, 2)) * 100;
	
	this->nbActionsExecuted++;
	this->totalReward += reward;

	this->setState(this->rng.getDouble(-2, 2));

}

bool RegressionWrapper::isCopyable() const
{
	return true;
}

Learn::LearningEnvironment* RegressionWrapper::clone() const
{
	return new RegressionWrapper(*this);
}

double RegressionWrapper::getScore() const
{

	return this->totalReward / (double)this->nbActionsExecuted;

}

bool RegressionWrapper::isTerminal() const
{
	return false;
}
