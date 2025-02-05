#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>
#include <algorithm>

#include "mujocoDoublePendulumWrapper.h"



bool MujocoDoublePendulumWrapper::isCopyable() const
{
	return true;
}

Learn::LearningEnvironment* MujocoDoublePendulumWrapper::clone() const
{
	return new MujocoDoublePendulumWrapper(*this);
}

double MujocoDoublePendulumWrapper::getScore() const
{
	return totalReward;
}

void MujocoDoublePendulumWrapper::reset(size_t seed, Learn::LearningMode mode, uint16_t iterationNumber, uint64_t generationNumber)
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

    std::mt19937 generator(rng.getInt32(0, 10000000)); // 42 est la graine
    std::normal_distribution<double> distribution(0, reset_noise_scale_);

	std::vector<double> qvel(m_->nv);
	for (size_t i = 0; i < qvel.size(); i++) {
		qvel[i] = init_qvel_[i] + distribution(generator);
	}
	mj_resetData(m_, d_);
	set_state(qpos, qvel);
	this->computeState();
	this->nbActionsExecuted = 0;
	this->totalReward = 0.0;
}

void MujocoDoublePendulumWrapper::doActions(std::vector<double> actionsID)
{
	do_simulation(actionsID, frame_skip_);
	this->computeState();


	// Get x and y from d_->site_xpos[0], site index 0
	double x = d_->site_xpos[0]; // x-coordinate
	double y = d_->site_xpos[2]; // z-coordinate (vertical axis)

	double dist_penalty = 0.01 * x * x + (y - 2.0) * (y - 2.0);

	double v1 = d_->qvel[1];
	double v2 = d_->qvel[2];

	double vel_penalty = 1e-3 * v1 * v1 + 5e-3 * v2 * v2;

	double alive_bonus = 10.0;

	double reward = alive_bonus - dist_penalty - vel_penalty;


	// Incremente the reward.
	this->totalReward += reward;

	this->nbActionsExecuted++;


}


bool MujocoDoublePendulumWrapper::isTerminal() const
{
        // Check for non-finite elements
        for (int i = 0; i < m_->nq; i++)
            if (!std::isfinite(d_->qpos[i]))
                return true;
        for (int i = 0; i < m_->nv; i++)
            if (!std::isfinite(d_->qvel[i]))
                return true;

        // Get y coordinate (vertical axis) of site[0], which is z-axis
        double y = d_->site_xpos[2]; // site_xpos[0] corresponds to site 0, index 2 is z-coordinate

        return (y <= 1.0);
    }




void MujocoDoublePendulumWrapper::computeState(){
		
	currentState.setDataAt(typeid(double), 0, d_->qpos[0]);

	currentState.setDataAt(typeid(double), 1, sin(d_->qpos[1]));
	currentState.setDataAt(typeid(double), 2, sin(d_->qpos[2]));

	currentState.setDataAt(typeid(double), 3, cos(d_->qpos[1]));
	currentState.setDataAt(typeid(double), 4, cos(d_->qpos[2]));

	// obs[5], obs[6], obs[7]: qvel[0], qvel[1], qvel[2], clipped to [-10,10]
	for (int i = 0; i < 3; i++) {
		currentState.setDataAt(typeid(double), 5 + i, std::max(std::min(d_->qvel[i], 10.0), -10.0));
	}

	// obs[8], obs[9], obs[10]: qfrc_constraint[0], [1], [2], clipped to [-10,10]
	for (int i = 0; i < 3; i++) {
		currentState.setDataAt(typeid(double), 8 + i, std::max(std::min(d_->qfrc_constraint[i], 10.0), -10.0));
	}

}

