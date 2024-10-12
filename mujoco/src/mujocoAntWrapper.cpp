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
	mj_resetData(m_, d_);
	set_state(qpos, qvel);
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

double MujocoAntWrapper::healthy_reward() {
	return static_cast<double>(is_healthy() || terminate_when_unhealthy_) *
			healthy_reward_;
}

double MujocoAntWrapper::control_cost(std::vector<double>& action) {
	double cost = 0;
	for (auto& a : action) cost += a * a;
	return control_cost_weight_ * cost;
}

std::vector<double> MujocoAntWrapper::contact_forces() {
	std::vector<double> forces;
	std::copy_n(d_->cfrc_ext, m_->nbody * 6, back_inserter(forces));
	for (auto& f : forces) {
		f = std::max(contact_force_range_[0],
						std::min(f, contact_force_range_[1]));
	}
	return forces;
}

double MujocoAntWrapper::contact_cost() {
	auto forces = contact_forces();
	double cost = 0;
	for (auto& f : forces) cost += f * f;
	return contact_cost_weight_ * cost;
}

bool MujocoAntWrapper::is_healthy() const{
	for (int i = 0; i < m_->nq; i++)
		if (!std::isfinite(d_->qpos[i])) return false;
	for (int i = 0; i < m_->nv; i++)
		if (!std::isfinite(d_->qvel[i])) return false;
	return (d_->qpos[2] >= healthy_z_range_[0] &&
			d_->qpos[2] <= healthy_z_range_[1]);
}

std::string MujocoAntWrapper::ExpandEnvVars(const std::string &str) {
	std::string result;
	size_t pos = 0;

	while (pos < str.length()) {
		if (str[pos] == '$') {
			size_t start = pos + 1;
			size_t end = start;

			// Handle ${VAR} format
			if (start < str.length() && str[start] == '{') {
				end = str.find('}', start);
				if (end != std::string::npos) {
					std::string varName =
						str.substr(start + 1, end - start - 1);
					const char *varValue = getenv(varName.c_str());
					if (varValue) {
						result += varValue;
					}
					pos = end + 1;
					continue;
				}
			}

			// Handle $VAR format
			while (end < str.length() &&
				(isalnum(str[end]) || str[end] == '_')) {
				++end;
			}
			std::string varName = str.substr(start, end - start);
			const char *varValue = getenv(varName.c_str());
			if (varValue) {
				result += varValue;
			}
			pos = end;
		} else {
			result += str[pos];
			++pos;
		}
	}
	return result;
}


