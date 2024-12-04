#ifndef MUJOCO_WRAPPER_H
#define MUJOCO_WRAPPER_H

#include <gegelati.h>
#include <mujoco.h>

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
	uint64_t stateSize;

public:


	/**
	* \brief Default constructor.
	*
	* Attributes angle and velocity are set to 0.0 by default.
	*/
	MujocoWrapper(uint64_t nbActions, uint64_t stateSize) :
		LearningEnvironment(nbActions, 0, false, nbActions),
		currentState{ stateSize }, stateSize{stateSize}
	{};

	/**
	* \brief Copy constructor for the MujocoWrapper.
	*
	* Default copy constructor since all attributes are trivially copyable.
	*/
	MujocoWrapper(const MujocoWrapper& other) : LearningEnvironment(other.nbContinuousAction, 0, false, other.nbContinuousAction),
		currentState{other.currentState}, stateSize{other.stateSize} {}
	

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

    void initialize_simulation();

    void set_state(std::vector<double>& qpos, std::vector<double>& qvel);

    void do_simulation(std::vector<double>& ctrl, int n_frames);

	Data::PrimitiveTypeArray<double>& getCurrentState();
	uint64_t getStateSize(){
		return stateSize;
	}

	std::string ExpandEnvVars(const std::string &str);


};

#endif // !MUJOCO_WRAPPER_H