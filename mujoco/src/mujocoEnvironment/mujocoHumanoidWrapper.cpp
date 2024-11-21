#include <math.h>
#include "mujocoHumanoidWrapper.h"
#include <array>


std::array<double, 2> MujocoHumanoidWrapper::massCenter() const
{
    // Calculate the center of mass by weighting each body's position by its mass
    double mass_sum = 0.0;
    double weighted_x_sum = 0.0;
    double weighted_y_sum = 0.0;

    for (int i = 0; i < m_->nbody; ++i) {
        // Weight each body's position by its mass in the x and y directions
        double mass = m_->body_mass[i];
        mass_sum += mass;
        weighted_x_sum += mass * d_->xpos[3 * i];      // xpos x-coordinate
        weighted_y_sum += mass * d_->xpos[3 * i + 1];  // xpos y-coordinate
    }

    // Return the center of mass in x and y directions
    return {weighted_x_sum / mass_sum, weighted_y_sum / mass_sum};
}


void MujocoHumanoidWrapper::reset(size_t seed, Learn::LearningMode mode, uint16_t iterationNumber, uint64_t generationNumber)
{
    size_t hash_seed = Data::Hash<size_t>()(seed) ^ Data::Hash<Learn::LearningMode>()(mode);
    if(mode == Learn::LearningMode::VALIDATION) {
        hash_seed = 6416846135168433 + iterationNumber;
    }
    this->rng.setSeed(hash_seed);

    std::vector<double> qpos(m_->nq);
    for (size_t i = 0; i < qpos.size(); i++) {
        qpos[i] = init_qpos_[i] + this->rng.getDouble(-reset_noise_scale_, reset_noise_scale_);
    }
    std::vector<double> qvel(m_->nv);
    for (size_t i = 0; i < qvel.size(); i++) {
        qvel[i] = init_qvel_[i] + this->rng.getDouble(-reset_noise_scale_, reset_noise_scale_);
    }
    mj_resetData(m_, d_);
    set_state(qpos, qvel);
    this->computeState();
    this->nbActionsExecuted = 0;
    this->totalReward = 0.0;
}

void MujocoHumanoidWrapper::doActions(std::vector<double> actionsID)
{
    // Record the initial center of mass position before the action
    auto com_before = massCenter();

    // Execute the simulation with the given actions
    do_simulation(actionsID, frame_skip_);

    // Record the center of mass position after the action and compute the velocity
    auto com_after = massCenter();
    auto x_velocity = (com_after[0] - com_before[0]) / m_->opt.timestep;

    // Calculate the control cost (penalizes large action magnitudes)
    auto ctrl_cost = control_cost(actionsID);

    // Calculate rewards
    auto forward_reward = forward_reward_weight_ * x_velocity;
    auto rewards = forward_reward;
    
    if (use_healthy_reward) {
        rewards += healthy_reward();
    }



    // Final reward for this step
    auto reward = rewards - ctrl_cost;

    // Update the state after executing the action
    this->computeState();

    // Increment the total reward and the count of executed actions
    this->totalReward += reward;
    this->nbActionsExecuted++;


}


bool MujocoHumanoidWrapper::isCopyable() const
{
    return true;
}

Learn::LearningEnvironment* MujocoHumanoidWrapper::clone() const
{
    return new MujocoHumanoidWrapper(*this);
}

double MujocoHumanoidWrapper::getScore() const
{
    return totalReward;
}

bool MujocoHumanoidWrapper::isTerminal() const
{
    return (terminate_when_unhealthy_ && !is_healthy());
}

double MujocoHumanoidWrapper::healthy_reward() {
    return static_cast<double>(is_healthy() || terminate_when_unhealthy_) * healthy_reward_;
}

double MujocoHumanoidWrapper::control_cost(std::vector<double>& action) {
    double cost = 0;
    for (auto& a : action) cost += a * a;
    return control_cost_weight_ * cost;
}

std::vector<double> MujocoHumanoidWrapper::contact_forces() {
    std::vector<double> forces;
    std::copy_n(d_->cfrc_ext, m_->nbody * 6, back_inserter(forces));
    for (auto& f : forces) {
        f = std::max(contact_force_range_[0], std::min(f, contact_force_range_[1]));
    }
    return forces;
}

double MujocoHumanoidWrapper::contact_cost() {
    auto forces = contact_forces();
    double cost = 0;
    for (auto& f : forces) cost += f * f;
    return contact_cost_weight_ * cost;
}

bool MujocoHumanoidWrapper::is_healthy() const {
    for (int i = 0; i < m_->nq; i++) if (!std::isfinite(d_->qpos[i])) return false;
    for (int i = 0; i < m_->nv; i++) if (!std::isfinite(d_->qvel[i])) return false;
    return (d_->qpos[2] >= healthy_z_range_[0] && d_->qpos[2] <= healthy_z_range_[1]);
}

void MujocoHumanoidWrapper::computeState() {
    int index = 0;


    // Get position (qpos)
    if (exclude_current_positions_from_observation_) {
        // Start from index 2 if excluding the first two positions
        for (int i = 2; i < m_->nq; i++) {
            currentState.setDataAt(typeid(double), index, d_->qpos[i]);
            index++;
        }
    } else {
        for (int i = 0; i < m_->nq; i++) {
            currentState.setDataAt(typeid(double), index, d_->qpos[i]);
            index++;
        }
    }

    // Get velocity (qvel)
    for (int i = 0; i < m_->nv; i++) {
        currentState.setDataAt(typeid(double), index, d_->qvel[i]);
        index++;
    }

    // Get center of mass inertia (cinert)
    for (int i = 0; i < 10 * m_->nbody; i++) {
        currentState.setDataAt(typeid(double), index, d_->cinert[i]);
        index++;
    }

    // Get center of mass velocity (cvel)
    for (int i = 0; i < 6 * m_->nbody; i++) {
        currentState.setDataAt(typeid(double), index, d_->cvel[i]);
        index++;
    }

    // Get actuator forces (qfrc_actuator)
    for (int i = 0; i < m_->nv; i++) {
        currentState.setDataAt(typeid(double), index, d_->qfrc_actuator[i]);
        index++;
    }


    // Get external contact forces (cfrc_ext)
    for (int i = 0; i < 6 * m_->nbody; i++) {
        currentState.setDataAt(typeid(double), index, d_->cfrc_ext[i]);
        index++;
    }
}
