#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/srv/set_model_configuration.hpp>
#include <std_srvs/srv/empty.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>

namespace gazebo
{
class SetModelConfigurationPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    initial_world_pose_ = model_->WorldPose();
    have_initial_world_pose_ = true;
    node_ = gazebo_ros::Node::Get(sdf);
    release_service_ = node_->create_service<std_srvs::srv::Empty>(
      "/gazebo/release_model_configuration",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        reset_hold_.store(false);
        RCLCPP_INFO(node_->get_logger(), "Dog reset hold released");
      });
    // Run after gazebo_ros2_control so a stale controller command cannot move
    // the model away from the reset pose in the same simulation step.
    update_connection_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&SetModelConfigurationPlugin::OnUpdate, this));
    reset_connection_ = event::Events::ConnectWorldReset(
      std::bind(&SetModelConfigurationPlugin::OnWorldReset, this));
    service_ = node_->create_service<gazebo_msgs::srv::SetModelConfiguration>(
      "/gazebo/set_model_configuration",
      [this](const std::shared_ptr<gazebo_msgs::srv::SetModelConfiguration::Request> request,
             std::shared_ptr<gazebo_msgs::srv::SetModelConfiguration::Response> response) {
        if (request->model_name != model_->GetName()) {
          response->status_message = "Model name does not match the spawned dog";
          return;
        }
        if (request->joint_names.size() != request->joint_positions.size() ||
            request->joint_names.empty()) {
          response->status_message = "Joint names and positions must have equal, nonzero lengths";
          return;
        }
        std::map<std::string, double> positions;
        for (size_t i = 0; i < request->joint_names.size(); ++i) {
          if (!model_->GetJoint(request->joint_names[i])) {
            response->status_message = "Unknown joint: " + request->joint_names[i];
            return;
          }
          positions[request->joint_names[i]] = request->joint_positions[i];
        }
        if (positions.size() != request->joint_names.size()) {
          response->status_message = "Duplicate joint name";
          return;
        }
        ApplyResetState(positions);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          initial_positions_ = positions;
        }
        reset_hold_.store(true);
        RCLCPP_INFO(node_->get_logger(), "Dog reset hold enabled");
        response->success = true;
        response->status_message = "Joint positions set";
      });
  }

private:
  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<gazebo_msgs::srv::SetModelConfiguration>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr release_service_;
  event::ConnectionPtr update_connection_;
  event::ConnectionPtr reset_connection_;
  std::mutex mutex_;
  std::map<std::string, double> initial_positions_;
  ignition::math::Pose3d initial_world_pose_;
  bool have_initial_world_pose_{false};
  std::atomic_bool reset_hold_{true};

  void ApplyResetState(const std::map<std::string, double> &positions)
  {
    model_->SetJointPositions(positions);
    // Position teleportation can leave the simulated joints with their
    // pre-reset velocity. Clear it together with the position so the pose
    // stays at the training reset state until explicit release.
    for (const auto &entry : positions) {
      const auto joint = model_->GetJoint(entry.first);
      if (joint) {
        joint->SetVelocity(0, 0.0);
      }
    }
    if (have_initial_world_pose_) {
      model_->SetWorldPose(initial_world_pose_);
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      model_->SetAngularVel(ignition::math::Vector3d::Zero);
    }
  }

  void OnUpdate()
  {
    if (!reset_hold_.load()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initial_positions_.empty()) {
      ApplyResetState(initial_positions_);
    }
  }

  void OnWorldReset()
  {
    // rl_sim's `r` key calls /reset_world.  Keep that reset at the training
    // pose until rl_sim explicitly releases it for the get-up sequence.
    reset_hold_.store(true);
  }
};
GZ_REGISTER_MODEL_PLUGIN(SetModelConfigurationPlugin)
}  // namespace gazebo
