#ifndef MUJOCOWRAPPER_H
#define MUJOCOWRAPPER_H

#include <gegelati.h>
#include <mujoco.h>
#include <GLFW/glfw3.h>

/**
* \brief Inverted pendulum LearningEnvironment.
*
* The code of the class is adapted from Florian Arrestier's code released
* under CECILL-C License.
* Link: https://github.com/preesm/preesm-apps/tree/master/org.ietr.preesm.reinforcement_learning
*/
class MujocoWrapper : public Learn::LearningEnvironment
{
protected:

	Data::PrimitiveTypeArray<double> currentState;

public:


	/**
	* \brief Default constructor.
	*
	* Attributes angle and velocity are set to 0.0 by default.
	*/
	MujocoWrapper(uint64_t nbActions, uint64_t stateSize, std::string actFunc) :
		LearningEnvironment(nbActions, 0, false, nbActions, actFunc),
		currentState{ stateSize }
	{};

	/**
	* \brief Copy constructor for the MujocoWrapper.
	*
	* Default copy constructor since all attributes are trivially copyable.
	*/
	MujocoWrapper(const MujocoWrapper& other) : LearningEnvironment(other.nbContinuousAction, 0, false, other.nbContinuousAction, other.activationFunction),
		currentState{other.currentState} {}
	

	/// Inherited via LearningEnvironment
	virtual std::vector<std::reference_wrapper<const Data::DataHandler>> getDataSources() override;

	// MuJoCo data structures
    mjModel* m_ = NULL;  // MuJoCo model
    mjData* d_ = NULL;   // MuJoCo data
    mjvCamera cam_;      // abstract camera
    mjvOption opt_;      // visualization options
    mjvScene scn_;       // abstract scene
    mjrContext con_;     // custom GPU context

    std::vector<double> init_qpos_;  // Initial positions
    std::vector<double> init_qvel_;  // Initial velocities

    std::string model_path_;  // Absolute path to model xml file
    int frame_skip_ = 1;  // Number of frames per simlation step
    int obs_size_;  // Number of variables in observation vector

    void initialize_simulation() {
        // Load and compile model
        char error[1000] = "Could not load binary model";
        m_ = mj_loadXML(model_path_.c_str(), 0, error, 1000);
        if (!m_) {
            mju_error(("Load model error: %s", error));
        }
        // Make data
        d_ = mj_makeData(m_);

        std::copy_n(d_->qpos, m_->nq, back_inserter(init_qpos_));
        std::copy_n(d_->qvel, m_->nv, back_inserter(init_qvel_));
    }

    void set_state(std::vector<double>& qpos, std::vector<double>& qvel) {
        // Set the joints position qpos and velocity qvel of the model.
        // Note: `qpos` and `qvel` is not the full physics state for all mujoco
        // models/environments
        // https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtstate

        for (int i = 0; i < m_->nq; i++) d_->qpos[i] = qpos[i];
        for (int i = 0; i < m_->nv; i++) d_->qvel[i] = qvel[i];
        mj_forward(m_, d_);
    }

	void computeState(){
		int index = 0;
        for (int i = 0; i < m_->nq; i++) 
		{
        	currentState.setDataAt(typeid(double), index, d_->qpos[i]);
			index++;
		}
        for (int i = 0; i < m_->nv; i++) 
		{
        	currentState.setDataAt(typeid(double), index, d_->qvel[i]);
			index++;
		}
	}

    void do_simulation(std::vector<double>& ctrl, int n_frames) {
        for (int i = 0; i < m_->nu; i++) {
            d_->ctrl[i] = ctrl[i];
        }
        for (int i = 0; i < n_frames; i++) {
            mj_step(m_, d_);
        }
        // As of MuJoCo 2.0, force - related quantities like cacc are not
        // computed unless there's a force sensor in the model. See https:
        // // github.com/openai/gym/issues/1541
        mj_rnePostConstraint(m_, d_);
    }

};

#endif // !MUJOCOWRAPPER_H