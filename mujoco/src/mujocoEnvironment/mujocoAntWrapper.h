#ifndef MUJOCO_ANT_WRAPPER_H
#define MUJOCO_ANT_WRAPPER_H

#include <gegelati.h>
#include "mujocoWrapper.h"

class MujocoAntWrapper : public MujocoWrapper
{
protected:

	/// Randomness control
	Mutator::RNG rng;

	/// Total reward accumulated since the last reset
	double totalReward = 0.0;

	/// Number of actions since the last reset
	uint64_t nbActionsExecuted = 0;

public:

	


	const std::string xmlFile;
    // Parameters
    double control_cost_weight_ = 0.5;
	bool use_healthy_reward;
    bool use_contact_forces_;
    double contact_cost_weight_ = 5e-4;
    double healthy_reward_ = 1.0;
    bool terminate_when_unhealthy_ = true;
    std::vector<double> healthy_z_range_;
    std::vector<double> contact_force_range_;
    double reset_noise_scale_ = 0.1;
    bool exclude_current_positions_from_observation_ = false;


	/**
	* \brief Default constructor.
	*
	* Attributes angle and velocity are set to 0.0 by default.
	*/
	MujocoAntWrapper(const char *pXmlFile, bool useHealthyReward=true, bool useContactForce=false, bool exclude_current_positions_from_observation = false) :
		MujocoWrapper(8, (exclude_current_positions_from_observation) ? 27:29), 
		xmlFile{pXmlFile}, use_healthy_reward{useHealthyReward}, use_contact_forces_{useContactForce},
		exclude_current_positions_from_observation_{exclude_current_positions_from_observation}
		{
			model_path_ = MujocoWrapper::ExpandEnvVars(xmlFile);
			healthy_z_range_ = {0.2, 1.0};
			contact_force_range_ = {-1.0, 1.0};
			initialize_simulation();
		};

    /**
    * \brief Copy constructor for the armLearnWrapper.
    */ 
    MujocoAntWrapper(const MujocoAntWrapper &other) : MujocoWrapper(other), 
	use_healthy_reward{other.use_healthy_reward}, use_contact_forces_{other.use_contact_forces_},
	exclude_current_positions_from_observation_{other.exclude_current_positions_from_observation_},
	xmlFile{other.xmlFile}
	{
		model_path_ = MujocoWrapper::ExpandEnvVars(other.xmlFile);
		healthy_z_range_ = {0.2, 1.0};
		contact_force_range_ = {-1.0, 1.0};
		initialize_simulation();
    }

    ~MujocoAntWrapper() {
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


    double healthy_reward();

    double control_cost(std::vector<double>& action);

    std::vector<double> contact_forces();

	void computeState();

    double contact_cost();

    bool is_healthy() const;


};

#endif // !MUJOCOANTWRAPPER_H