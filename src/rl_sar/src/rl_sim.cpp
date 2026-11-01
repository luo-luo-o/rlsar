/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim.hpp"

RL_Sim::RL_Sim(int argc, char **argv)
{
#if defined(USE_ROS1)
    this->ang_vel_axis = "world";
    ros::NodeHandle nh;
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    nh.param<std::string>("robot_name", this->robot_name, "");
#elif defined(USE_ROS2)
    ros2_node = std::make_shared<rclcpp::Node>("rl_sim_node");
    this->ang_vel_axis = "body";
    this->ros_namespace = ros2_node->get_namespace();
    // get params from param_node
    param_client = ros2_node->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};
    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(ros2_node->get_node_base_interface(), future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }
#endif

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
#if defined(USE_ROS1)
    this->joint_publishers_commands.resize(this->params.Get<int>("num_of_dofs"));
#elif defined(USE_ROS2)
    this->robot_command_publisher_msg.motor_command.resize(this->params.Get<int>("num_of_dofs"));
    this->robot_state_subscriber_msg.motor_state.resize(this->params.Get<int>("num_of_dofs"));
#endif
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

#if defined(USE_ROS1)
    auto joint_controller_names_vec = this->params.Get<std::vector<std::string>>("joint_controller_names");  // avoid dangling reference
    this->StartJointController(this->ros_namespace, joint_controller_names_vec);
    // publisher
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const std::string &joint_controller_name = joint_controller_names_vec[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/command";
        this->joint_publishers[joint_controller_name] =
            nh.advertise<robot_msgs::MotorCommand>(topic_name, 10);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);
    this->joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &RL_Sim::JoyCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const std::string &joint_controller_name = joint_controller_names_vec[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/state";
        this->joint_subscribers[joint_controller_name] =
            nh.subscribe<robot_msgs::MotorState>(topic_name, 10,
                [this, joint_controller_name](const robot_msgs::MotorState::ConstPtr &msg)
                {
                    this->JointStatesCallback(msg, joint_controller_name);
                }
            );
        this->joint_positions[joint_controller_name] = 0.0f;
        this->joint_velocities[joint_controller_name] = 0.0f;
        this->joint_efforts[joint_controller_name] = 0.0f;
    }

    // service
    nh.param<std::string>("gazebo_model_name", this->gazebo_model_name, "");
    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    this->gazebo_reset_world_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
#elif defined(USE_ROS2)
    // The dog launch starts joint_state_broadcaster only after its Gazebo
    // reset succeeds. Its first joint state is the startup barrier for this
    // controller, which is still owned by rl_sim.
    if (this->robot_name == "dog")
    {
        const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
        bool joints_ready = false;
        auto joint_state_waiter = ros2_node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SystemDefaultsQoS(),
            [&joints_ready, &joint_names](const sensor_msgs::msg::JointState::SharedPtr msg) {
                joints_ready = std::all_of(joint_names.begin(), joint_names.end(),
                    [&msg](const std::string &name) {
                        return std::find(msg->name.begin(), msg->name.end(), name) != msg->name.end();
                    });
            });
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
        while (rclcpp::ok() && !joints_ready && std::chrono::steady_clock::now() < deadline)
        {
            rclcpp::spin_some(ros2_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!joints_ready)
        {
            throw std::runtime_error("Timed out waiting for initialized dog joint states");
        }
    }
    // The dog launch owns its controller lifecycle; starting another spawner
    // here collides with the controller already loaded by Gazebo.
    if (this->robot_name != "dog")
    {
        this->StartJointController(this->ros_namespace, this->params.Get<std::vector<std::string>>("joint_names"));
    }
    // publisher
    this->robot_command_publisher = ros2_node->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
    this->joy_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Joy::SharedPtr msg) {this->JoyCallback(msg);}
    );
    this->gazebo_imu_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(), [this] (const sensor_msgs::msg::Imu::SharedPtr msg) {this->GazeboImuCallback(msg);}
    );
    if (this->robot_name == "dog")
    {
        this->dog_base_state_subscriber = ros2_node->create_subscription<nav_msgs::msg::Odometry>(
            "/dog/base_state", rclcpp::SystemDefaultsQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->DogBaseStateCallback(msg); }
        );
        const auto contact_topics = this->params.Get<std::vector<std::string>>("foot_contact_topics");
        if (contact_topics.size() != dog_contact_subscribers.size())
        {
            throw std::runtime_error("dog policy requires exactly four foot contact topics");
        }
        for (size_t i = 0; i < contact_topics.size(); ++i)
        {
            dog_contact_subscribers[i] = ros2_node->create_subscription<gazebo_msgs::msg::ContactsState>(
                contact_topics[i], rclcpp::SystemDefaultsQoS(),
                [this, i](const gazebo_msgs::msg::ContactsState::SharedPtr msg) { this->GazeboContactCallback(msg, i); }
            );
        }
    }
    this->robot_state_subscriber = ros2_node->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this] (const robot_msgs::msg::RobotState::SharedPtr msg) {this->RobotStateCallback(msg);}
    );

    // service
    this->gazebo_pause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/pause_physics");
    this->gazebo_unpause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/unpause_physics");
    this->gazebo_reset_world_client = ros2_node->create_client<std_srvs::srv::Empty>("/reset_world");
    if (this->robot_name == "dog")
    {
        this->dog_set_model_configuration_client =
            ros2_node->create_client<gazebo_msgs::srv::SetModelConfiguration>(
                "/gazebo/set_model_configuration");
        this->dog_release_model_configuration_client =
            ros2_node->create_client<std_srvs::srv::Empty>(
                "/gazebo/release_model_configuration");
        // The Gazebo plugin holds the training pose until the user starts the
        // dog FSM with key 0.  This prevents a passive controller from
        // collapsing the robot before the get-up sequence begins.
        this->dog_reset_hold.store(true);
    }

    if (this->robot_name != "dog")
    {
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
    }
#endif

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

RL_Sim::~RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names)
{
#if defined(USE_ROS1)
    pid_t pid0 = fork();
    if (pid0 == 0)
    {
        std::string cmd = "rosrun controller_manager spawner joint_state_controller ";
        for (const auto& name : names)
        {
            cmd += name + " ";
        }
        cmd += "__ns:=" + ros_namespace;
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
#elif defined(USE_ROS2)
    const char* ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
        {
            throw std::runtime_error("Failed to create temporary parameter file");
        }

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";
        for (const auto& name : names)
        {
            tmp_file << "            - " << name << "\n";
        }
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        {
            throw std::runtime_error("Failed to start joint controller");
        }

        std::filesystem::remove(tmp_path);
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
#endif
}

void RL_Sim::GetState(RobotState<float> *state)
{
#if defined(USE_ROS1)
    const auto &orientation = this->pose.orientation;
    const auto &angular_velocity = this->vel.angular;
#elif defined(USE_ROS2)
    if (this->robot_name == "dog" && this->dog_model_state_received.load())
    {
        std::lock_guard<std::mutex> lock(this->dog_observation_mutex);
        for (size_t i = 0; i < 4; ++i) state->imu.quaternion[i] = this->dog_base_quat[i];
        for (size_t i = 0; i < 3; ++i) state->imu.gyroscope[i] = this->dog_base_ang_vel[i];
    }
    else
    {
        const auto &orientation = this->gazebo_imu.orientation;
        const auto &angular_velocity = this->gazebo_imu.angular_velocity;
        state->imu.quaternion[0] = orientation.w;
        state->imu.quaternion[1] = orientation.x;
        state->imu.quaternion[2] = orientation.y;
        state->imu.quaternion[3] = orientation.z;
        state->imu.gyroscope[0] = angular_velocity.x;
        state->imu.gyroscope[1] = angular_velocity.y;
        state->imu.gyroscope[2] = angular_velocity.z;
    }
#endif

#if defined(USE_ROS1)
    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;
#endif

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
#if defined(USE_ROS1)
        state->motor_state.q[i] = this->joint_positions[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
        state->motor_state.dq[i] = this->joint_velocities[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
        state->motor_state.tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
#elif defined(USE_ROS2)
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est;
#endif
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
#if defined(USE_ROS1)
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = command->motor_command.kp[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = command->motor_command.kd[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
#elif defined(USE_ROS2)
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = command->motor_command.kp[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = command->motor_command.kd[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
#endif
    }

#if defined(USE_ROS1)
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->joint_publishers[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]].publish(this->joint_publishers_commands[i]);
    }
#elif defined(USE_ROS2)
    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
#endif
}

void RL_Sim::RobotControl()
{
#if defined(USE_ROS2)
    if (this->robot_name == "dog" &&
        (!this->dog_state_received.load() || !this->dog_model_state_received.load()))
    {
        return;
    }

    // After a dog reset, the Gazebo model plugin holds the training pose until
    // rl_sim explicitly releases it. Keep rl_sim quiet during that hold so the
    // policy's previous command cannot immediately overwrite the reset.
    if (this->robot_name == "dog" && this->dog_reset_hold.load() &&
        this->control.current_keyboard != Input::Keyboard::R &&
        this->control.current_gamepad != Input::Gamepad::RB_Y)
    {
        if (this->control.current_keyboard == Input::Keyboard::Num0 ||
            this->control.current_gamepad == Input::Gamepad::A)
        {
            this->dog_reset_hold.store(false);
            this->ReleaseDogResetHold();
            this->fsm.RequestStateChange("RLFSMStateGetUp");
        }
        else
        {
            this->control.ClearInput();
            return;
        }
    }
#endif
    this->GetState(&this->robot_state);

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
#if defined(USE_ROS2)
        if (this->robot_name == "dog")
        {
            this->dog_reset_hold.store(true);
            this->rl_init_done = false;
            this->fsm.RequestStateChange("RLFSMStatePassive");
        }
#endif
#if defined(USE_ROS1)
        std_srvs::Empty empty;
        this->gazebo_reset_world_client.call(empty);
#elif defined(USE_ROS2)
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        if (this->robot_name == "dog")
        {
            // Reapply the training pose after the world reset has completed;
            // the plugin remains held until key 0 starts the get-up sequence.
            this->gazebo_reset_world_client->async_send_request(
                empty_request,
                [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
                    this->ResetDogJoints();
                });
        }
        else
        {
            auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
        }
#endif
        this->control.current_keyboard = this->control.last_keyboard;
        this->control.ClearInput();
        return;
    }

    this->StateController(&this->robot_state, &this->robot_command);
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_pause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_unpause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

#if defined(USE_ROS2)
void RL_Sim::ResetDogJoints()
{
    if (!this->dog_set_model_configuration_client)
    {
        return;
    }
    auto request = std::make_shared<gazebo_msgs::srv::SetModelConfiguration::Request>();
    request->model_name = this->gazebo_model_name;
    request->urdf_param_name = "robot_description";
    request->joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    const auto init_angle = this->params.Get<std::vector<float>>("init_angle");
    request->joint_positions.assign(init_angle.begin(), init_angle.end());
    this->dog_set_model_configuration_client->async_send_request(
        request,
        [this, init_angle](rclcpp::Client<gazebo_msgs::srv::SetModelConfiguration>::SharedFuture future) {
            const auto response = future.get();
            if (!response->success)
            {
                std::cerr << LOGGER::ERROR << "Dog reset joint configuration failed: "
                          << response->status_message << std::endl;
            }
            else
            {
                // The ros2_control controller keeps applying its last effort
                // command even while rl_sim is quiet.  Replace that command
                // with a PD target at the training reset pose so the model
                // reset and the active controller agree on the same state.
                robot_msgs::msg::RobotCommand reset_command;
                reset_command.motor_command.resize(this->params.Get<int>("num_of_dofs"));
                const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
                const auto fixed_kp = this->params.Get<std::vector<float>>("fixed_kp");
                const auto fixed_kd = this->params.Get<std::vector<float>>("fixed_kd");
                for (size_t i = 0; i < init_angle.size(); ++i)
                {
                    const auto output_index = static_cast<size_t>(joint_mapping[i]);
                    reset_command.motor_command[output_index].q = init_angle[i];
                    reset_command.motor_command[output_index].dq = 0.0F;
                    reset_command.motor_command[output_index].kp = fixed_kp[i];
                    reset_command.motor_command[output_index].kd = fixed_kd[i];
                    reset_command.motor_command[output_index].tau = 0.0F;
                }
                this->robot_command_publisher->publish(reset_command);
                auto pause_request = std::make_shared<std_srvs::srv::Empty::Request>();
                this->gazebo_pause_physics_client->async_send_request(pause_request);
                std::cout << LOGGER::INFO << "Dog reset joint configuration applied" << std::endl;
            }
        });
}

void RL_Sim::ReleaseDogResetHold()
{
    if (!this->dog_release_model_configuration_client)
    {
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    this->dog_release_model_configuration_client->async_send_request(request);
    auto unpause_request = std::make_shared<std_srvs::srv::Empty::Request>();
    this->gazebo_unpause_physics_client->async_send_request(unpause_request);
}
#endif

#if defined(USE_ROS1)
void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    this->vel = msg->twist[2];
    this->pose = msg->pose[2];
}
#elif defined(USE_ROS2)
void RL_Sim::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->gazebo_imu = *msg;
}

void RL_Sim::DogBaseStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto &orientation = msg->pose.pose.orientation;
    const std::vector<float> quat{static_cast<float>(orientation.w), static_cast<float>(orientation.x),
                                  static_cast<float>(orientation.y), static_cast<float>(orientation.z)};
    const auto &twist = msg->twist.twist;
    const auto lin_vel_body = QuatRotateInverse(quat, {
        static_cast<float>(twist.linear.x), static_cast<float>(twist.linear.y), static_cast<float>(twist.linear.z)});
    const auto ang_vel_body = QuatRotateInverse(quat, {
        static_cast<float>(twist.angular.x), static_cast<float>(twist.angular.y), static_cast<float>(twist.angular.z)});

    std::lock_guard<std::mutex> lock(this->dog_observation_mutex);
    this->dog_base_quat = {quat[0], quat[1], quat[2], quat[3]};
    std::copy(lin_vel_body.begin(), lin_vel_body.end(), this->dog_base_lin_vel.begin());
    std::copy(ang_vel_body.begin(), ang_vel_body.end(), this->dog_base_ang_vel.begin());
    this->dog_model_state_received.store(true);
}

void RL_Sim::GazeboContactCallback(const gazebo_msgs::msg::ContactsState::SharedPtr msg, size_t foot_index)
{
    if (foot_index >= this->dog_foot_contacts.size()) return;

    const float threshold = this->params.Get<float>("foot_contact_threshold", 2.0F);
    bool in_contact = false;
    for (const auto &contact : msg->states)
    {
        const auto &force = contact.total_wrench.force;
        const float magnitude = std::sqrt(static_cast<float>(force.x * force.x + force.y * force.y + force.z * force.z));
        if (magnitude > threshold)
        {
            in_contact = true;
            break;
        }
    }

    std::lock_guard<std::mutex> lock(this->dog_observation_mutex);
    this->dog_foot_contacts[foot_index] = in_contact ? 1.0F : 0.0F;
}
#endif

void RL_Sim::CmdvelCallback(
#if defined(USE_ROS1)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}

void RL_Sim::JoyCallback(
#if defined(USE_ROS1)
    const sensor_msgs::Joy::ConstPtr &msg
#elif defined(USE_ROS2)
    const sensor_msgs::msg::Joy::SharedPtr msg
#endif
)
{
    this->joy_msg = *msg;

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5, DPadX=6, DPadY=7

    if (this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4]) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->joy_msg.axes[1]; // LY
    this->control.y = this->joy_msg.axes[0]; // LX
    this->control.yaw = this->joy_msg.axes[3]; // RX
}

#if defined(USE_ROS1)
void RL_Sim::JointStatesCallback(const robot_msgs::MotorState::ConstPtr &msg, const std::string &joint_controller_name)
{
    this->joint_positions[joint_controller_name] = msg->q;
    this->joint_velocities[joint_controller_name] = msg->dq;
    this->joint_efforts[joint_controller_name] = msg->tau_est;
}
#elif defined(USE_ROS2)
void RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    this->robot_state_subscriber_msg = *msg;
    if (this->robot_name == "dog" &&
        msg->motor_state.size() == static_cast<size_t>(this->params.Get<int>("num_of_dofs")))
    {
        this->dog_state_received.store(true);
    }
}
#endif

void RL_Sim::RunModel()
{
    if (this->robot_name == "dog" && !this->dog_model_state_received.load()) return;

    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        if (this->robot_name == "dog")
        {
            this->obs.commands[0] = this->control.y;
            this->obs.commands[1] = -this->control.x;
            this->obs.commands[2] = this->control.yaw;
            if (this->control.navigation_mode)
            {
                this->obs.commands[0] = static_cast<float>(this->cmd_vel.linear.y);
                this->obs.commands[1] = -static_cast<float>(this->cmd_vel.linear.x);
                this->obs.commands[2] = static_cast<float>(this->cmd_vel.angular.z);
            }
            std::lock_guard<std::mutex> lock(this->dog_observation_mutex);
            this->obs.lin_vel = {this->dog_base_lin_vel[0], this->dog_base_lin_vel[1], this->dog_base_lin_vel[2]};
            this->obs.foot_contacts.assign(this->dog_foot_contacts.begin(), this->dog_foot_contacts.end());
        }
        else
        {
            this->obs.commands[0] = this->control.x;
            this->obs.commands[1] = this->control.y;
            this->obs.commands[2] = this->control.yaw;
            if (this->control.navigation_mode)
            {
                this->obs.commands[0] = static_cast<float>(this->cmd_vel.linear.x);
                this->obs.commands[1] = static_cast<float>(this->cmd_vel.linear.y);
                this->obs.commands[2] = static_cast<float>(this->cmd_vel.angular.z);
            }
        }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
#if defined(USE_ROS1)
        this->plot_real_joint_pos[i].push_back(this->joint_positions[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]]);
        this->plot_target_joint_pos[i].push_back(this->joint_publishers_commands[i].q);
#elif defined(USE_ROS2)
        this->plot_real_joint_pos[i].push_back(this->robot_state_subscriber_msg.motor_state[i].q);
        this->plot_target_joint_pos[i].push_back(this->robot_command_publisher_msg.motor_command[i].q);
#endif
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

#if defined(USE_ROS1)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Sim rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Sim>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#endif
    return 0;
}
