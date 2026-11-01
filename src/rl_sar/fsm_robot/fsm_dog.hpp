#ifndef DOG_FSM_HPP
#define DOG_FSM_HPP
#include "fsm_go2.hpp"
namespace dog_fsm {
class RLFSMStateGetUp : public go2_fsm::RLFSMStateGetUp {
public:
    explicit RLFSMStateGetUp(RL *rl) : go2_fsm::RLFSMStateGetUp(rl) {
        // The dog policy is trained from this reset pose.  The inherited Go2
        // trajectory (1.36, -2.65) is outside the dog's joint limits.
        pre_running_pos = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.018f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.057f
        };
    }

    void Run() override {
        if (stand_from_passive) {
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q,
                            pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            // Keep the training reset pose.  A dog-specific policy expects
            // the two nonzero shank angles in its first observation.
            Interpolate(percent_getup, pre_running_pos, pre_running_pos,
                        0.1f, "Getting up", true);
        } else {
            Interpolate(percent_getup, rl.now_state.motor_state.q,
                        pre_running_pos, 1.0f, "Getting up", true);
        }
    }
};

class RLFSMStateRLLocomotion : public go2_fsm::RLFSMStateRLLocomotion {
public:
    explicit RLFSMStateRLLocomotion(RL *rl) : go2_fsm::RLFSMStateRLLocomotion(rl) {}
    void Enter() override { rl.config_name = "slow_walk"; rl.episode_length_buf = 0; try { rl.InitRL(rl.robot_name + "/" + rl.config_name); rl.now_state = *fsm_state; } catch (...) { rl.rl_init_done = false; rl.fsm.RequestStateChange("RLFSMStatePassive"); } }
};
}
class DogFSMFactory : public FSMFactory {
public:
    explicit DogFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string& n) override { RL *rl=static_cast<RL*>(context); if(n=="RLFSMStatePassive") return std::make_shared<go2_fsm::RLFSMStatePassive>(rl); if(n=="RLFSMStateGetUp") return std::make_shared<dog_fsm::RLFSMStateGetUp>(rl); if(n=="RLFSMStateGetDown") return std::make_shared<go2_fsm::RLFSMStateGetDown>(rl); if(n=="RLFSMStateRLLocomotion") return std::make_shared<dog_fsm::RLFSMStateRLLocomotion>(rl); return nullptr; }
    std::string GetType() const override { return "dog"; }
    std::vector<std::string> GetSupportedStates() const override { return {"RLFSMStatePassive","RLFSMStateGetUp","RLFSMStateGetDown","RLFSMStateRLLocomotion"}; }
    std::string GetInitialState() const override { return initial_state_; }
private: std::string initial_state_;
};
REGISTER_FSM_FACTORY(DogFSMFactory, "RLFSMStatePassive")
#endif
