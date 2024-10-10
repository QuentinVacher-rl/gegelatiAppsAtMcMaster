#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "mujocoAntWrapper.h"




void MujocoAntWrapper::reset(size_t seed, Learn::LearningMode mode, uint16_t iterationNumber, uint64_t generationNumber)
{
	// Create seed from seed and mode
	size_t hash_seed = Data::Hash<size_t>()(seed) ^ Data::Hash<Learn::LearningMode>()(mode);
	if(mode == Learn::LearningMode::VALIDATION){
		hash_seed = 6416846135168433;
	}

	// Reset the RNG
	this->rng.setSeed(hash_seed);


	std::vector<double> qpos(m_->nq);
	for (size_t i = 0; i < qpos.size(); i++) {
		qpos[i] = init_qpos_[i] + this->rng.getDouble(-reset_noise_scale_, reset_noise_scale_);
	}
	std::vector<double> qvel(m_->nv);
	for (size_t i = 0; i < qvel.size(); i++) {
		qvel[i] = init_qvel_[i] + this->rng.getDouble(0.0, reset_noise_scale_);
	}
	set_state(qpos, qvel);
	mj_resetData(m_, d_);
	this->computeState();
	this->nbActionsExecuted = 0;
	this->totalReward = 0.0;
}

void MujocoAntWrapper::doActions(std::vector<double> actionsID)
{
	auto x_pos_before = d_->qpos[0];
	do_simulation(actionsID, frame_skip_);
	auto x_pos_after = d_->qpos[0];
	auto x_vel = (x_pos_after - x_pos_before) / m_->opt.timestep;
	auto forward_reward = x_vel;
	auto rewards = forward_reward + healthy_reward();
	auto ctrl_cost = control_cost(actionsID);
	auto costs = ctrl_cost;
	if (use_contact_forces_) {
		costs += contact_cost();
	}
	auto reward = rewards - costs;

	this->computeState();

	// Incremente the reward.
	this->totalReward += reward;

	this->nbActionsExecuted = 0;
}

bool MujocoAntWrapper::isCopyable() const
{
	return true;
}

Learn::LearningEnvironment* MujocoAntWrapper::clone() const
{
	return new MujocoAntWrapper(*this);
}

double MujocoAntWrapper::getScore() const
{
	return totalReward;
}

bool MujocoAntWrapper::isTerminal() const
{
	return (terminate_when_unhealthy_ && !is_healthy());
}


