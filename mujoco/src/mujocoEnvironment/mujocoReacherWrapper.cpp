#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "mujocoReacherWrapper.h"




void MujocoReacherWrapper::reset(size_t seed, Learn::LearningMode mode, uint16_t iterationNumber, uint64_t generationNumber)
{
	// Create seed from seed and mode
	size_t hash_seed = Data::Hash<size_t>()(seed) ^ Data::Hash<Learn::LearningMode>()(mode);
	if(mode == Learn::LearningMode::VALIDATION){
		hash_seed = 6416846135168433+iterationNumber;
	}

	// Reset the RNG
	this->rng.setSeed(hash_seed);


	std::vector<double> qpos(m_->nq);
	for (size_t i = 0; i < 2; i++) {
		qpos[i] = init_qpos_[i] + this->rng.getDouble(-reset_noise_scale_pos, reset_noise_scale_pos);
	}
	std::vector<double> qvel(m_->nv - 2);
	for (size_t i = 0; i < 2; i++) {
		qvel[i] = init_qvel_[i] + this->rng.getDouble(-reset_noise_scale_vel, reset_noise_scale_vel);
	}


	std::vector<double> goal(2);
	do {
		
		goal[0] = this->rng.getDouble(-0.2, 0.2);
		goal[1] = this->rng.getDouble(-0.2, 0.2);
		
	
	} while (goal[0] * goal[0] + goal[1] * goal[1] >= 0.04);

 	std::copy_n(goal.begin(), 2, qpos.end() - 2 );
	std::fill_n(std::back_inserter(qvel), 2, 0.0);
	mj_resetData(m_, d_);
	set_state(qpos, qvel);
	this->computeState();
	this->nbActionsExecuted = 0;
	this->totalReward = 0.0;
}




void MujocoReacherWrapper::computeState() {
    int index = 0;

    // Get theta = qpos[0:2]
    std::vector<double> theta(2);
    for (int i = 0; i < 2; ++i) {
        theta[i] = d_->qpos[i];
    }

    // Compute cos(theta) and sin(theta)
    std::vector<double> cos_theta(2), sin_theta(2);
    for (int i = 0; i < 2; ++i) {
        cos_theta[i] = cos(theta[i]);
        sin_theta[i] = sin(theta[i]);
    }

    // Add cos(theta) to the current state
    for (double cos_val : cos_theta) {
        currentState.setDataAt(typeid(double), index++, cos_val);
    }

    // Add sin(theta) to the current state
    for (double sin_val : sin_theta) {
        currentState.setDataAt(typeid(double), index++, sin_val);
    }

    // Add qpos[2:] to the current state
    for (int i = 2; i < m_->nq; ++i) {
        currentState.setDataAt(typeid(double), index++, d_->qpos[i]);
    }

    // Add qvel[0:2] to the current state
    for (int i = 0; i < 2; ++i) {
        currentState.setDataAt(typeid(double), index++, d_->qvel[i]);
    }

    // Calculate and add xpos differences to the current state
    std::vector<double> dist_diff = {
        *(d_->xpos + 3 * 3) - *(d_->xpos + 3 * 4),
        *(d_->xpos + 3 * 3 + 1) - *(d_->xpos + 3 * 4 + 1)
    };

    for (double dist_val : dist_diff) {
        currentState.setDataAt(typeid(double), index++, dist_val);
    }
}


void MujocoReacherWrapper::doActions(std::vector<double> actionsID)
{
        //auto vec = d_->xipos;
	// xipos id : fingertip = 3 , target = 4
	std::vector<double> dist_diff = {*(d_->xpos + 3 * 3) - *(d_->xpos + 3 * 4) , 
		*(d_->xpos + 3 * 3 + 1) - *(d_->xpos + 3 * 4 + 1) };
	//std::cout << dist_di
	double reward_dist = -reward_distance_weight * std::sqrt(std::pow(dist_diff[0],2) + std::pow(dist_diff[1],2));
//	std::cout << "reward_dist "<< reward_dist << std::endl; 
	double reward_ctrl = -control_cost(actionsID);
//	std::cout << "reward ctrl" << reward_ctrl << std::endl;
	auto reward = reward_dist + reward_ctrl;
	do_simulation(actionsID, frame_skip_);

	this->computeState();

	// Incremente the reward.
	this->totalReward += reward;

	this->nbActionsExecuted++;


}


bool MujocoReacherWrapper::isCopyable() const
{
	return true;
}

Learn::LearningEnvironment* MujocoReacherWrapper::clone() const
{
	return new MujocoReacherWrapper(*this);
}

double MujocoReacherWrapper::getScore() const
{
	return totalReward;
}

bool MujocoReacherWrapper::isTerminal() const
{
	// Reacher is never terminal, only max number of actions
	return false;
}


double MujocoReacherWrapper::control_cost(std::vector<double>& action) {
	double cost = 0;
	for (auto& a : action) cost += a * a;
	return reward_control_weight * cost;
}




