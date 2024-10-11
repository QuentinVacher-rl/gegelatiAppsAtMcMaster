#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "mujocoWrapper.h"


std::vector<std::reference_wrapper<const Data::DataHandler>> MujocoWrapper::getDataSources()
{
	auto result = std::vector<std::reference_wrapper<const Data::DataHandler>>();
	result.push_back(this->currentState);
	return result;
}


void MujocoWrapper::initialize_simulation() {
	// Load and compile model
	char error[1000] = "Could not load binary model";
	m_ = mj_loadXML(model_path_.c_str(), 0, error, 1000);
	if (!m_) {
		char formattedError[256];
		sprintf(formattedError, "Load model error: %s", error);
		mju_error(formattedError);
	}
	// Make data
	d_ = mj_makeData(m_);

	std::copy_n(d_->qpos, m_->nq, back_inserter(init_qpos_));
	std::copy_n(d_->qvel, m_->nv, back_inserter(init_qvel_));
}

void MujocoWrapper::set_state(std::vector<double>& qpos, std::vector<double>& qvel) {
	// Set the joints position qpos and velocity qvel of the model.
	// Note: `qpos` and `qvel` is not the full physics state for all mujoco
	// models/environments
	// https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtstate

	for (int i = 0; i < m_->nq; i++) d_->qpos[i] = qpos[i];
	for (int i = 0; i < m_->nv; i++) d_->qvel[i] = qvel[i];
	mj_forward(m_, d_);
}

void MujocoWrapper::computeState(){
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

void MujocoWrapper::do_simulation(std::vector<double>& ctrl, int n_frames) {
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
