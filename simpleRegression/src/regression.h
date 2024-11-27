#ifndef PENDULUM_H
#define PENDULUM_H

#include <gegelati.h>

/**
* \brief Inverted pendulum LearningEnvironment.
*
* The code of the class is adapted from Florian Arrestier's code released
* under CECILL-C License.
* Link: https://github.com/preesm/preesm-apps/tree/master/org.ietr.preesm.reinforcement_learning
*/
class RegressionWrapper : public Learn::LearningEnvironment
{
private:
	/**
	* \brief Available actions for the LearningAgent.
	*
	* Each number $a$ in this list, with $a \in ]0.0;1.0], corresponds to two
	* actions available for the LearningAgent: $a*MAX_TORQUE$ and
	* $-a*MAX_TORQUE$.
	* An additional action 0.0 is always available to the LearningAgent.
	*
	* A total of availableAction.size()*2 + 1 actions are thus available to
	* the LearningAgent, through the doAction() method.
	*/
	const std::vector<double> availableActions;

	/// Randomness control
	Mutator::RNG rng;

	/// Total reward accumulated since the last reset
	double totalReward = 0.0;

	/// Number of actions since the last reset
	uint64_t nbActionsExecuted = 0;

	/// Copy of current angle and velocity provided to the LearningAgent
	/// Current angle of the pendulum in [-M_PI; M_PI]
	/// Current velocity of the pendulum in [-1;1]
	Data::PrimitiveTypeArray<double> currentState;

	bool velocityAvailable;

	double currentVelocity = 0;

	bool isValidation = false;

protected:
	/// Setter for angle state
	void setState(double newValue);
	/// Setter for angle state
	double getState() const;

public:

	/**
	* \brief Default constructor.
	*
	* Attributes angle and velocity are set to 0.0 by default.
	*/
	RegressionWrapper() :
		LearningEnvironment(10, 0, false, 1), // see availableActions comment.
		currentState{ 1 }
	{};

	/**
	* \brief Copy constructor for the Pendulum.
	*
	* Default copy constructor since all attributes are trivially copyable.
	*/
	RegressionWrapper(const RegressionWrapper& other) = default;


	/// Inherited via LearningEnvironment
	virtual std::vector<std::reference_wrapper<const Data::DataHandler>> getDataSources() override;

	/// Inherited via LearningEnvironment
	virtual void reset(size_t seed = 0, Learn::LearningMode mode = Learn::LearningMode::TRAINING,
					   uint16_t iterationNumber = 0, uint64_t generationNumber = 0) override;

	/// Inherited via LearningEnvironment
	virtual void doActions(std::vector<double> actionsID) override;

	/// Inherited via LearningEnvironment
	virtual bool isCopyable() const override;

	/// Inherited via LearningEnvironment
	virtual LearningEnvironment* clone() const;

	/**
	* \brief Get a score for the pendulum stabilization.
	*
	* The score returned at any time can either be positive or negative.
	*
	* A positive score is returned if the pendulum has been stabilized, that is,
	* the isTerminal() method returns true.
	* In such a case, the returned score will be $10 / ln(nbActionExecuted)$
	* such that shorter convergence time leads to higher scores.
	*
	* A negative score is returned if the pendulum has not been stabilized
	* (yet).
	* In such a case, the returned score simply is the average reward since
	* the last reset.
	*
	* \return a double value corresponding to the score.
	*/
	virtual double getScore() const override;

	/**
	* \brief Is the pendulum considered stabilized.
	*
	* If the mean reward over the recent rewardHistory is lower than a fixed
	* threshold, then the pendulum is considered to be stable in the upward
	* position and the learningAgent has succeded in stabilizing it.
	*
	* \return true if the pendulum has been stabilized, false otherwise.
	*/
	virtual bool isTerminal() const override;
};

#endif // !PENDULUM_H