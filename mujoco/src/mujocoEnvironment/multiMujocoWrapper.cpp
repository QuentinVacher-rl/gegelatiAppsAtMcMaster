
#include "multiMujocoWrapper.h"

uint64_t MultiMujocoWrapper::getMaxActionContinuous(std::vector<MujocoWrapper*>* wrappersUsed)
{
	if(wrappersUsed == nullptr){
		wrappersUsed = &this->wrappers;
	}
	uint64_t maxSize = 0;
	for(auto wrapper: *wrappersUsed){
		maxSize = std::max(maxSize, wrapper->getNbContinuousAction());
	}
	return maxSize;
}

uint64_t MultiMujocoWrapper::getMaxCurrentStateSize(std::vector<MujocoWrapper*>* wrappersUsed)
{
	if(wrappersUsed == nullptr){
		wrappersUsed = &this->wrappers;
	}
	uint64_t maxSize = 0;
	for(auto wrapper: *wrappersUsed){
		maxSize = std::max(maxSize, wrapper->getStateSize());
	}
	return maxSize;
}


bool MultiMujocoWrapper::isCopyable() const
{
    return true;
}

Learn::LearningEnvironment* MultiMujocoWrapper::clone() const
{
	return new MultiMujocoWrapper(*this);
}


/// Inherited via LearningEnvironment
std::vector<std::reference_wrapper<const Data::DataHandler>> MultiMujocoWrapper::getDataSources()
{
	auto result = std::vector<std::reference_wrapper<const Data::DataHandler>>();
	result.push_back(this->currentState);
	return result;
}

void MultiMujocoWrapper::computeState(){

	MujocoWrapper* wrapper = wrappers.at(currentWrapper);

	auto wrapperState = wrapper->getCurrentState();
	for(size_t i = 0; i < wrapper->getStateSize(); i++){
		double value = *wrapperState.getDataAt(typeid(double), i).getSharedPointer<double>().get();
		currentState.setDataAt(typeid(double), i, value);
	}

	for(size_t i = wrapper->getStateSize(); i < this->getMaxCurrentStateSize(); i++){
		currentState.setDataAt(typeid(double), i, 0.0);
	}
}



void MultiMujocoWrapper::doActions(std::vector<double> vectActionID)
{
	vectActionID.resize(wrappers.at(currentWrapper)->getNbContinuousAction());
	wrappers.at(currentWrapper)->doActions(vectActionID);
	nbActionsExecuted++;

	if(nbActionsExecuted == nbActionsPerEnv.at(currentWrapper) || wrappers.at(currentWrapper)->isTerminal()){
		currentWrapper++;
		nbActionsExecuted = 0;
	}

	if(currentWrapper < this->wrappers.size()){
		this->computeState();
	}

}

void MultiMujocoWrapper::reset(size_t seed,
							  Learn::LearningMode mode,
							  uint16_t iterationNumber,
							  uint64_t generationNumber)
{
	currentWrapper = 0;
	nbActionsExecuted = 0;
	for(auto wrapper: wrappers){
		wrapper->reset(seed, mode, iterationNumber, generationNumber);
	}
	computeState();
	
}

double MultiMujocoWrapper::getScore() const
{
	double score = 0;
	for(auto wrapper: wrappers){
		score += wrapper->getScore();
	}
	return score/wrappers.size();
}

bool MultiMujocoWrapper::isTerminal() const
{
	return (currentWrapper == wrappers.size());
}

const MujocoWrapper* MultiMujocoWrapper::getWrapperAt(uint64_t idx) const
{
	return wrappers.at(idx);
}

const uint64_t MultiMujocoWrapper::getNbWrapper() const
{
	return wrappers.size();
}