#ifndef MUJOCOHUMANOIDWRAPPER_H
#define MUJOCOHUMANOIDWRAPPER_H

#include <gegelati.h>
#include "mujocoWrapper.h"

class MujocoHumanoidWrapper : public MujocoWrapper
{
protected:
protected:
    Mutator::RNG rng;
    double totalReward = 0.0;
    uint64_t nbActionsExecuted = 0;
    const std::string xmlFile;

public:
    double control_cost_weight_ = 0.1;
    double forward_reward_weight_ = 1.25;
    double healthy_reward_ = 5.0;
    bool terminate_when_unhealthy_ = true;
    bool use_healthy_reward;
    bool use_contact_forces_;
    double contact_cost_weight_ = 5e-4;
    std::vector<double> healthy_z_range_ = {1.0, 2.0};
    std::vector<double> contact_force_range_ = {-1.0, 1.0};
    double reset_noise_scale_ = 1e-2;
    bool exclude_current_positions_from_observation_;

    MujocoHumanoidWrapper(const char *pXmlFile, 
                          bool useHealthyReward = true, 
                          bool useContactForce = false,
                          bool exclude_current_positions_from_observation = false) :
        MujocoWrapper(17, exclude_current_positions_from_observation ? 376 : 378),
        xmlFile{pXmlFile},
        use_healthy_reward{useHealthyReward},
        use_contact_forces_{useContactForce},
        exclude_current_positions_from_observation_{exclude_current_positions_from_observation}
    {
        model_path_ = ExpandEnvVars(xmlFile);
        initialize_simulation();
    }

    MujocoHumanoidWrapper(const MujocoHumanoidWrapper &other) :
        MujocoWrapper(other),
        xmlFile{other.xmlFile},
        forward_reward_weight_{other.forward_reward_weight_},
        control_cost_weight_{other.control_cost_weight_},
        healthy_reward_{other.healthy_reward_},
        terminate_when_unhealthy_{other.terminate_when_unhealthy_},
        healthy_z_range_{other.healthy_z_range_},
        reset_noise_scale_{other.reset_noise_scale_},
        exclude_current_positions_from_observation_{other.exclude_current_positions_from_observation_},
        use_healthy_reward{other.use_healthy_reward},
        use_contact_forces_{other.use_contact_forces_}
    {
        model_path_ = ExpandEnvVars(other.xmlFile);
        initialize_simulation();
    }


    ~MujocoHumanoidWrapper() {
        mj_deleteData(d_);
        mj_deleteModel(m_);
#if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
#endif
    }

    virtual void reset(size_t seed = 0, Learn::LearningMode mode = Learn::LearningMode::TRAINING, uint16_t iterationNumber = 0, uint64_t generationNumber = 0) override;
    virtual void doActions(std::vector<double> actionsID) override;
    virtual bool isCopyable() const override;
    virtual LearningEnvironment* clone() const override;
    virtual double getScore() const override;
    virtual bool isTerminal() const override;

    double healthy_reward();
    double control_cost(std::vector<double>& action);
    std::vector<double> contact_forces();
    double contact_cost();
    bool is_healthy() const;
    std::string ExpandEnvVars(const std::string &str);
    std::array<double, 2> massCenter() const;
    virtual void computeState() override;
};

#endif // MUJOCOHUMANOIDWRAPPER_H
