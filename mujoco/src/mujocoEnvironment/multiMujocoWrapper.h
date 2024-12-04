#ifndef MULTI_MUJOCO_WRAPPER_H
#define MULTI_MUJOCO_WRAPPER_H

#include <gegelati.h>
#include <mujoco.h>

#include "mujocoAntWrapper.h"
#include "mujocoHalfCheetahWrapper.H"

/**
* \brief Inverted pendulum LearningEnvironment.
*
* The code of the class is adapted from Florian Arrestier's code released
* under CECILL-C License.
* Link: https://github.com/preesm/preesm-apps/tree/master/org.ietr.preesm.reinforcement_learning
*/
class MultiMujocoWrapper : public Learn::LearningEnvironment
{
protected:


	Data::PrimitiveTypeArray<double> currentState;

	/// Number of actions since the last reset
	uint64_t nbActionsExecuted = 0;

	uint64_t currentWrapper = 0;

	std::vector<MujocoWrapper*> wrappers;
	std::vector<uint64_t> nbActionsPerEnv;

public:

	/**
	* \brief Default constructor.
	*
	* Attributes angle and velocity are set to 0.0 by default.
	*/
	MultiMujocoWrapper(std::vector<MujocoWrapper*>& wrappersUsed, std::vector<uint64_t>& nbActionsEachEnv) :
		wrappers{wrappersUsed}, nbActionsPerEnv{nbActionsEachEnv},
		LearningEnvironment(this->getMaxActionContinuous(&wrappersUsed), 0, false, this->getMaxActionContinuous(&wrappersUsed)), 
		currentState{this->getMaxCurrentStateSize(&wrappersUsed)}
	{};
	/**
	* \brief Copy constructor for the MujocoWrapper.
	*
	* Default copy constructor since all attributes are trivially copyable.
	*/
	MultiMujocoWrapper(const MultiMujocoWrapper& other) : LearningEnvironment(other.nbContinuousAction, 0, false, other.nbContinuousAction),
		currentState{other.currentState}, nbActionsPerEnv{other.nbActionsPerEnv} {
			for(auto* wrapper: other.wrappers){
				wrappers.push_back(dynamic_cast<MujocoWrapper*>(wrapper->clone()));
			}
		}

	uint64_t getMaxActionContinuous(std::vector<MujocoWrapper*>* wrappersUsed=nullptr);
	uint64_t getMaxCurrentStateSize(std::vector<MujocoWrapper*>* wrappersUsed=nullptr);

	/// Inherited via LearningEnvironment
	virtual std::vector<std::reference_wrapper<const Data::DataHandler>> getDataSources() override;

	void computeState();

	virtual void doActions(std::vector<double> vectActionID);

	virtual void reset(size_t seed = 0,
					   Learn::LearningMode mode = Learn::LearningMode::TRAINING,
					   uint16_t iterationNumber = 0,
					   uint64_t generationNumber = 0);

    virtual bool isCopyable() const override;
    virtual LearningEnvironment* clone() const override;
	virtual double getScore() const override;

	virtual bool isTerminal() const override;

	const MujocoWrapper* getWrapperAt(uint64_t idx) const;

	const uint64_t getNbWrapper() const;
};

#endif // !MULTI_MUJOCO_WRAPPER_H