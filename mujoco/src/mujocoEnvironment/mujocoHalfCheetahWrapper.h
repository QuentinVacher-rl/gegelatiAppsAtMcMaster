#ifndef MUJOCO_HALF_CHEETAH_WRAPPER_H
#define MUJOCO_HALF_CHEETAH_WRAPPER_H

#include <gegelati.h>
#include "mujocoWrapper.h"

class MujocoHalfCheetahWrapper : public MujocoWrapper
{
protected:

	/// Randomness control
	Mutator::RNG rng;

	/// Total reward accumulated since the last reset
	double totalReward = 0.0;

	/// Number of actions since the last reset
	uint64_t nbActionsExecuted = 0;

	const std::string xmlFile;

public:

    // Parameters
    double forward_reward_weight = 1.0;
    double control_cost_weight_ = 0.5;
    double reset_noise_scale_ = 0.1;
    bool exclude_current_positions_from_observation_ = true;


	MujocoHalfCheetahWrapper(const char *pXmlFile, bool exclude_current_positions_from_observation = true) :
		MujocoWrapper(6, (exclude_current_positions_from_observation) ? 17:18), xmlFile{pXmlFile},
		exclude_current_positions_from_observation_{exclude_current_positions_from_observation}
		{
			model_path_ = MujocoWrapper::ExpandEnvVars(xmlFile);
			initialize_simulation();

		};

    /**
    * \brief Copy constructor for the armLearnWrapper.
    */ 
    MujocoHalfCheetahWrapper(const MujocoHalfCheetahWrapper &other) : MujocoWrapper(other),
	exclude_current_positions_from_observation_{other.exclude_current_positions_from_observation_}
	{   
		model_path_ = MujocoWrapper::ExpandEnvVars(other.xmlFile);
		initialize_simulation();
    }

    ~MujocoHalfCheetahWrapper() {
        // Free visualization storage
        //mjv_freeScene(&scn_);
        //mjr_freeContext(&con_);

        // Free MuJoCo model and data
        mj_deleteData(d_);
        mj_deleteModel(m_);

        // Terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
#endif
    }

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

    double control_cost(std::vector<double>& action);


	void computeState();



};

#endif // !MUJOCO_HALF_CHEETAH_WRAPPER_H