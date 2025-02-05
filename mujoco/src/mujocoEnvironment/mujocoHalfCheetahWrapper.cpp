#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "mujocoHalfCheetahWrapper.h"




void MujocoHalfCheetahWrapper::reset(size_t seed, Learn::LearningMode mode, uint16_t iterationNumber, uint64_t generationNumber)
{
	// Create seed from seed and mode
	size_t hash_seed = Data::Hash<size_t>()(seed) ^ Data::Hash<Learn::LearningMode>()(mode);
	if(mode == Learn::LearningMode::VALIDATION){
		hash_seed = 6416846135168433+iterationNumber;
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
	mj_resetData(m_, d_);
	set_state(qpos, qvel);
	this->computeState();
	this->nbActionsExecuted = 0;
	this->totalReward = 0.0;
}

void MujocoHalfCheetahWrapper::doActions(std::vector<double> actionsID)
{
	auto x_pos_before = d_->qpos[0];
	do_simulation(actionsID, frame_skip_);
	auto x_pos_after = d_->qpos[0];
	auto x_vel = (x_pos_after - x_pos_before) / m_->opt.timestep;

	auto forward_reward = forward_reward_weight * x_vel;
	auto rewards = forward_reward;
	
	auto costs = control_cost(actionsID);

	auto reward = rewards - costs;

	this->computeState();

	// Incremente the reward.
	this->totalReward += reward;

	this->nbActionsExecuted++;


}

bool MujocoHalfCheetahWrapper::isCopyable() const
{
	return true;
}

Learn::LearningEnvironment* MujocoHalfCheetahWrapper::clone() const
{
	return new MujocoHalfCheetahWrapper(*this);
}

double MujocoHalfCheetahWrapper::getScore() const
{
	return totalReward;
}

bool MujocoHalfCheetahWrapper::isTerminal() const
{
	return false;
}

double MujocoHalfCheetahWrapper::control_cost(std::vector<double>& action) {
	double cost = 0;
	for (auto& a : action) cost += a * a;
	return control_cost_weight_ * cost;
}


void MujocoHalfCheetahWrapper::computeState(){
	int index = 0;
	int reduction = 0;
	if(exclude_current_positions_from_observation_){
		reduction = 1;
	}
	for (int i = reduction; i < m_->nq; i++) 
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

