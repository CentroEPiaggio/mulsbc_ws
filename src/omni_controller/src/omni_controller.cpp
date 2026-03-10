#include "omni_controller/omni_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"

#include <cmath>
#include <algorithm>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace omni_controller
{

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn OmniController::on_init()
{
    try
    {
        // Wheel joint positions (map: position_key → joint name(s))
        // Mecanum: LF, LH, RF, RH (single string each)
        // Differential: LEFT, RIGHT (string array each — supports multiple wheels per side)
        auto_declare<std::string>("wheel_joints.LF", "");
        auto_declare<std::string>("wheel_joints.LH", "");
        auto_declare<std::string>("wheel_joints.RF", "");
        auto_declare<std::string>("wheel_joints.RH", "");
        auto_declare<std::vector<std::string>>("wheel_joints.LEFT", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("wheel_joints.RIGHT", std::vector<std::string>());

        // Other joint lists
        auto_declare<std::vector<std::string>>("leg_joints", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("distributor_names", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("second_encoder_joints", std::vector<std::string>());

        // Wheel IK params
        auto_declare<std::string>("feet_type", "none");
        auto_declare<double>("wheel_rad", 0.05);
        auto_declare<double>("driveshaft_x", 0.235);
        auto_declare<double>("driveshaft_y", 0.188);
        auto_declare<double>("mecanum_angle", 135.0);
        auto_declare<double>("track_width", 0.4);

        // Communication
        auto_declare<int>("input_frequency", 100);
        auto_declare<bool>("BestEffort_QOS", true);

        // Feature flags
        auto_declare<bool>("sim", false);
        auto_declare<bool>("pub_odom", false);
        auto_declare<bool>("pub_performance", true);

        // Homing
        auto_declare<std::vector<double>>("homing_phases", std::vector<double>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Exception during parameter declaration: %s", e.what());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "on_init successful");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_configure(const rclcpp_lifecycle::State &)
{
    // ── Read parameters ─────────────────────────────────────────────────
    std::string feet_type = get_node()->get_parameter("feet_type").as_string();

    // Build wheel_joints_ (flat list) and wheel_groups_ (IK index → joints)
    wheel_joints_.clear();
    wheel_groups_.clear();
    if (feet_type == "mecanum")
    {
        const std::vector<std::string> positions = {"LF", "LH", "RF", "RH"};
        for (const auto & pos : positions)
        {
            std::string jnt = get_node()->get_parameter("wheel_joints." + pos).as_string();
            if (jnt.empty())
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "wheel_joints.%s is required for feet_type='mecanum'", pos.c_str());
                return CallbackReturn::ERROR;
            }
            wheel_joints_.push_back(jnt);
            wheel_groups_.push_back({jnt});
        }
    }
    else if (feet_type == "differential")
    {
        const std::vector<std::string> positions = {"LEFT", "RIGHT"};
        for (const auto & pos : positions)
        {
            auto joints = get_node()->get_parameter("wheel_joints." + pos).as_string_array();
            if (joints.empty())
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "wheel_joints.%s requires at least one joint for feet_type='differential'",
                             pos.c_str());
                return CallbackReturn::ERROR;
            }
            wheel_groups_.push_back(joints);
            wheel_joints_.insert(wheel_joints_.end(), joints.begin(), joints.end());
        }
    }

    leg_joints_            = get_node()->get_parameter("leg_joints").as_string_array();
    distributor_names_     = get_node()->get_parameter("distributor_names").as_string_array();
    second_encoder_joints_ = get_node()->get_parameter("second_encoder_joints").as_string_array();
    sim_flag_        = get_node()->get_parameter("sim").as_bool();
    pub_odom_        = get_node()->get_parameter("pub_odom").as_bool();
    pub_performance_ = get_node()->get_parameter("pub_performance").as_bool();

    has_wheels_       = !wheel_joints_.empty();
    has_legs_         = !leg_joints_.empty();
    has_distributors_ = !distributor_names_.empty();

    // ── Build combined motor joint list ─────────────────────────────────
    all_motor_joints_.clear();
    all_motor_joints_.insert(all_motor_joints_.end(),
                             wheel_joints_.begin(), wheel_joints_.end());
    all_motor_joints_.insert(all_motor_joints_.end(),
                             leg_joints_.begin(), leg_joints_.end());

    // Build second-encoder flags per motor joint
    se_flag_.resize(all_motor_joints_.size(), false);
    for (size_t i = 0; i < all_motor_joints_.size(); i++)
    {
        for (const auto & se : second_encoder_joints_)
        {
            if (all_motor_joints_[i] == se)
            {
                se_flag_[i] = true;
                break;
            }
        }
    }

    // ── Validate and create WheelIK ─────────────────────────────────────
    if (has_wheels_)
    {
        try
        {
            wheel_ik_ = create_wheel_ik(feet_type);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "WheelIK creation failed: %s", e.what());
            return CallbackReturn::ERROR;
        }

        if (wheel_ik_)
        {
            WheelIKConfig ik_config;
            ik_config.wheel_rad     = get_node()->get_parameter("wheel_rad").as_double();
            ik_config.driveshaft_x  = get_node()->get_parameter("driveshaft_x").as_double();
            ik_config.driveshaft_y  = get_node()->get_parameter("driveshaft_y").as_double();
            ik_config.mecanum_angle = get_node()->get_parameter("mecanum_angle").as_double();
            ik_config.track_width   = get_node()->get_parameter("track_width").as_double();

            try
            {
                // For differential with multi-wheel groups, pass one name per group
                // (IK only cares about count, not the actual names)
                std::vector<std::string> ik_names;
                for (const auto & group : wheel_groups_)
                    ik_names.push_back(group.front());
                wheel_ik_->configure(ik_config, ik_names);
            }
            catch (const std::exception & e)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "WheelIK configure failed: %s", e.what());
                return CallbackReturn::ERROR;
            }
        }
        else if (feet_type == "none")
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "wheel_joints is non-empty but feet_type='none'. Wheels will not receive IK commands.");
        }
    }

    // ── Initialize leg command buffers ───────────────────────────────────
    for (const auto & jnt : leg_joints_)
    {
        leg_pos_cmd_[jnt] = 0.0;
        leg_vel_cmd_[jnt] = 0.0;
        leg_eff_cmd_[jnt] = 0.0;
        leg_kp_cmd_[jnt]  = 1.0;
        leg_kd_cmd_[jnt]  = 1.0;
    }

    // ── Homing configuration ────────────────────────────────────────────
    homing_phase_durations_ = get_node()->get_parameter("homing_phases").as_double_array();
    if (homing_phase_durations_.empty() || !has_legs_)
    {
        has_homing_ = false;
    }
    else
    {
        if (homing_phase_durations_.size() != 2)
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "homing_phases must have exactly 2 entries, got %zu",
                         homing_phase_durations_.size());
            return CallbackReturn::ERROR;
        }
        has_homing_ = true;

        for (const auto & jnt : leg_joints_)
        {
            auto_declare<double>("homing_config." + jnt + ".q0", 0.0);
            auto_declare<double>("homing_config." + jnt + ".q1", 0.0);
            auto_declare<double>("homing_config." + jnt + ".q2", 0.0);

            JointHomingConfig cfg;
            cfg.q0 = get_node()->get_parameter("homing_config." + jnt + ".q0").as_double();
            cfg.q1 = get_node()->get_parameter("homing_config." + jnt + ".q1").as_double();
            cfg.q2 = get_node()->get_parameter("homing_config." + jnt + ".q2").as_double();
            homing_config_[jnt] = cfg;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Homing configured for %zu leg joints", leg_joints_.size());
    }

    // ── QoS setup ───────────────────────────────────────────────────────
    bool best_effort = get_node()->get_parameter("BestEffort_QOS").as_bool();
    int input_freq   = get_node()->get_parameter("input_frequency").as_int();

    // ── Subscribers ─────────────────────────────────────────────────────
    if (has_wheels_ && wheel_ik_)
    {
        rclcpp::QoS twist_qos(10);
        if (best_effort)
            twist_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        milliseconds deadline_dur{input_freq + 5};
        twist_qos.deadline(deadline_dur);

        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.event_callbacks.deadline_callback =
            [this](rclcpp::QOSDeadlineRequestedInfo &)
        {
            dl_miss_count_++;
            if (dl_miss_count_ > 10)
            {
                std::lock_guard<std::mutex> lg(var_mutex_);
                c_stt_ = ControllerState::INACTIVE;
            }
            RCLCPP_WARN(get_node()->get_logger(), "Twist deadline missed");
        };

        twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "~/twist_cmd", twist_qos,
            std::bind(&OmniController::twist_callback, this, std::placeholders::_1),
            sub_opt);
    }

    if (has_legs_)
    {
        rclcpp::QoS legs_qos(5);
        legs_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        legs_sub_ = get_node()->create_subscription<JointsCommand>(
            "~/legs_cmd", legs_qos,
            std::bind(&OmniController::legs_callback, this, std::placeholders::_1));
    }

    // ── Publishers ──────────────────────────────────────────────────────
    stt_pub_ = get_node()->create_publisher<JointsStates>("~/joints_state", 5);

    if (pub_performance_)
        per_pub_ = get_node()->create_publisher<PacketPass>("~/performance", 5);

    if (has_distributors_)
        dist_pub_ = get_node()->create_publisher<DistributorsState>("~/distributors_state", 5);

    if (pub_odom_ && has_wheels_ && wheel_ik_)
        odom_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("~/odom", 10);

    // ── Services ────────────────────────────────────────────────────────
    activate_srv_ = get_node()->create_service<TransactionService>(
        "~/activate_srv",
        std::bind(&OmniController::activate_service_cb, this,
                  std::placeholders::_1, std::placeholders::_2));

    emergency_srv_ = get_node()->create_service<TransactionService>(
        "~/emergency_srv",
        std::bind(&OmniController::emergency_service_cb, this,
                  std::placeholders::_1, std::placeholders::_2));

    if (has_homing_)
    {
        homing_srv_ = get_node()->create_service<TransactionService>(
            "~/homing_srv",
            std::bind(&OmniController::homing_service_cb, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    // ── Pre-allocate messages ───────────────────────────────────────────
    size_t n_motors = all_motor_joints_.size();

    stt_msg_.name.resize(n_motors);
    stt_msg_.position.resize(n_motors);
    stt_msg_.velocity.resize(n_motors);
    stt_msg_.effort.resize(n_motors);
    stt_msg_.temperature.resize(n_motors);
    stt_msg_.current.resize(n_motors);

    // Only allocate sec_enc arrays if any secondary encoders exist
    bool has_sec_enc = std::any_of(se_flag_.begin(), se_flag_.end(), [](bool v) { return v; });
    if (has_sec_enc)
    {
        stt_msg_.sec_enc_pos.resize(n_motors, 0.0);
        stt_msg_.sec_enc_vel.resize(n_motors, 0.0);
    }

    if (pub_performance_)
    {
        per_msg_.name.resize(n_motors);
        per_msg_.pack_loss.resize(n_motors);
        for (size_t i = 0; i < n_motors; i++)
            per_msg_.name[i] = all_motor_joints_[i];
    }

    if (has_distributors_)
    {
        size_t n_dist = distributor_names_.size();
        dist_msg_.name.resize(n_dist);
        dist_msg_.voltage.resize(n_dist);
        dist_msg_.current.resize(n_dist);
        dist_msg_.temperature.resize(n_dist);
    }

    RCLCPP_INFO(get_node()->get_logger(), "on_configure successful (wheels=%zu, legs=%zu, dist=%zu)",
                wheel_joints_.size(), leg_joints_.size(), distributor_names_.size());
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_activate(const rclcpp_lifecycle::State &)
{
    // Build command index map
    cmd_idx_.clear();
    for (size_t i = 0; i < command_interfaces_.size(); i++)
    {
        std::string key = command_interfaces_[i].get_prefix_name() + "/" +
                          command_interfaces_[i].get_interface_name();
        cmd_idx_[key] = i;
    }

    // Build state index map
    stt_idx_.clear();
    for (size_t i = 0; i < state_interfaces_.size(); i++)
    {
        std::string key = state_interfaces_[i].get_prefix_name() + "/" +
                          state_interfaces_[i].get_interface_name();
        stt_idx_[key] = i;
    }

    // Start in INACTIVE — wait for activate_srv call
    c_stt_ = ControllerState::INACTIVE;
    dl_miss_count_ = 0;

    RCLCPP_INFO(get_node()->get_logger(), "on_activate successful (INACTIVE, waiting for activate_srv)");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_deactivate(const rclcpp_lifecycle::State &)
{
    c_stt_ = ControllerState::INACTIVE;
    zero_all_commands();
    RCLCPP_INFO(get_node()->get_logger(), "on_deactivate");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_cleanup(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Interface Configuration
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::InterfaceConfiguration OmniController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jnt : all_motor_joints_)
    {
        cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        if (!sim_flag_)
        {
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_POSITION);
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_EFFORT);
            cfg.names.push_back(jnt + "/" + hw_if::KP_SCALE);
            cfg.names.push_back(jnt + "/" + hw_if::KD_SCALE);
        }
    }
    return cfg;
}

controller_interface::InterfaceConfiguration OmniController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // System-level performance interfaces
    std::string sys_name = "Pi3Hat";
    cfg.names.push_back(sys_name + "/" + hw_if::VALIDITY_LOSS);
    cfg.names.push_back(sys_name + "/" + hw_if::CYCLE_DUR);

    // Per-motor package loss
    for (const auto & jnt : all_motor_joints_)
        cfg.names.push_back(jnt + "/" + hw_if::PACKAGE_LOSS);

    // Per-motor state
    for (const auto & jnt : all_motor_joints_)
    {
        cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_POSITION);
        cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_EFFORT);
        cfg.names.push_back(jnt + "/" + hw_if::TEMPERATURE);
        cfg.names.push_back(jnt + "/" + hw_if::Q_CURRENT);
    }

    // Second encoder state
    for (size_t i = 0; i < all_motor_joints_.size(); i++)
    {
        if (se_flag_[i])
        {
            cfg.names.push_back(all_motor_joints_[i] + "_second_encoder/" +
                                hardware_interface::HW_IF_POSITION);
            cfg.names.push_back(all_motor_joints_[i] + "_second_encoder/" +
                                hardware_interface::HW_IF_VELOCITY);
        }
    }

    // Distributor state
    for (const auto & dist : distributor_names_)
    {
        cfg.names.push_back(dist + "/" + hw_if::VOLTAGE);
        cfg.names.push_back(dist + "/" + hw_if::CURRENT);
        cfg.names.push_back(dist + "/" + hw_if::TEMPERATURE);
    }

    return cfg;
}

// ═══════════════════════════════════════════════════════════════════════════
//  update() — main control loop
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::return_type OmniController::update(
    const rclcpp::Time & time, const rclcpp::Duration &)
{
    // Phase 1: Read and publish state
    publish_joint_states(time);
    if (pub_performance_ && per_pub_)
        publish_performance(time);
    if (has_distributors_ && dist_pub_)
        publish_distributor_states(time);
    if (pub_odom_ && has_wheels_ && wheel_ik_ && odom_pub_)
        publish_odometry(time);

    // Phase 2: Process commands
    std::lock_guard<std::mutex> lg(var_mutex_);
    switch (c_stt_)
    {
    case ControllerState::INACTIVE:
        zero_all_commands();
        break;
    case ControllerState::HOMING:
        update_homing(time);
        break;
    case ControllerState::ACTIVE:
        if (has_wheels_ && wheel_ik_)
            write_wheel_commands();
        if (has_legs_)
            write_leg_commands();
        break;
    }

    return controller_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Callbacks
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    dl_miss_count_ = 0;
    if (c_stt_ == ControllerState::ACTIVE)
    {
        base_vel_[0] = msg->linear.x;
        base_vel_[1] = msg->linear.y;
        base_vel_[2] = msg->angular.z;
    }
}

void OmniController::legs_callback(const JointsCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (c_stt_ == ControllerState::HOMING)
        return;
    for (size_t i = 0; i < msg->name.size(); i++)
    {
        const auto & name = msg->name[i];
        if (leg_pos_cmd_.count(name))
        {
            if (i < msg->position.size())  leg_pos_cmd_[name] = msg->position[i];
            if (i < msg->velocity.size())  leg_vel_cmd_[name] = msg->velocity[i];
            if (i < msg->effort.size())    leg_eff_cmd_[name] = msg->effort[i];
            if (i < msg->kp_scale.size())  leg_kp_cmd_[name]  = msg->kp_scale[i];
            if (i < msg->kd_scale.size())  leg_kd_cmd_[name]  = msg->kd_scale[i];
        }
    }
}

void OmniController::activate_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (c_stt_ == ControllerState::INACTIVE && req->data)
    {
        c_stt_ = ControllerState::ACTIVE;
        res->success = true;
        res->message = "Active mode activated";
    }
    else
    {
        res->success = false;
        res->message = req->data ? "Not in Inactive mode" : "Invalid request";
    }
}

void OmniController::emergency_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (c_stt_ != ControllerState::INACTIVE && req->data)
    {
        c_stt_ = ControllerState::INACTIVE;
        res->success = true;
        res->message = "Emergency mode activated";
    }
    else
    {
        res->success = false;
        res->message = req->data ? "Already in Emergency mode" : "Invalid request";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  State publishing
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::publish_joint_states(const rclcpp::Time & time)
{
    stt_msg_.header.set__stamp(time);
    for (size_t i = 0; i < all_motor_joints_.size(); i++)
    {
        const auto & jnt = all_motor_joints_[i];
        stt_msg_.name[i]        = jnt;
        stt_msg_.position[i]    = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
        stt_msg_.velocity[i]    = get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        stt_msg_.effort[i]      = get_state(jnt + "/" + hardware_interface::HW_IF_EFFORT);
        stt_msg_.temperature[i] = get_state(jnt + "/" + hw_if::TEMPERATURE);
        stt_msg_.current[i]     = get_state(jnt + "/" + hw_if::Q_CURRENT);

        if (se_flag_[i])
        {
            stt_msg_.sec_enc_pos[i] = get_state(jnt + "_second_encoder/" +
                                                 hardware_interface::HW_IF_POSITION);
            stt_msg_.sec_enc_vel[i] = get_state(jnt + "_second_encoder/" +
                                                 hardware_interface::HW_IF_VELOCITY);
        }
    }
    stt_pub_->publish(stt_msg_);
}

void OmniController::publish_performance(const rclcpp::Time & time)
{
    per_msg_.header.set__stamp(time);
    per_msg_.set__valid(get_state("Pi3Hat/" + std::string(hw_if::VALIDITY_LOSS)));
    per_msg_.set__cycle_dur(get_state("Pi3Hat/" + std::string(hw_if::CYCLE_DUR)));

    for (size_t i = 0; i < all_motor_joints_.size(); i++)
    {
        per_msg_.pack_loss[i] = get_state(all_motor_joints_[i] + "/" +
                                          std::string(hw_if::PACKAGE_LOSS));
    }
    per_pub_->publish(per_msg_);
}

void OmniController::publish_distributor_states(const rclcpp::Time & time)
{
    dist_msg_.header.set__stamp(time);
    for (size_t i = 0; i < distributor_names_.size(); i++)
    {
        const auto & name = distributor_names_[i];
        dist_msg_.name[i]        = name;
        dist_msg_.voltage[i]     = get_state(name + "/" + std::string(hw_if::VOLTAGE));
        dist_msg_.current[i]     = get_state(name + "/" + std::string(hw_if::CURRENT));
        dist_msg_.temperature[i] = get_state(name + "/" + std::string(hw_if::TEMPERATURE));
    }
    dist_pub_->publish(dist_msg_);
}

void OmniController::publish_odometry(const rclcpp::Time & time)
{
    // Build one velocity per IK slot by averaging all wheels in each group
    std::vector<double> wheel_vels(wheel_groups_.size());
    for (size_t g = 0; g < wheel_groups_.size(); g++)
    {
        double sum = 0.0;
        for (const auto & jnt : wheel_groups_[g])
            sum += get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        wheel_vels[g] = sum / static_cast<double>(wheel_groups_[g].size());
    }

    auto odom = wheel_ik_->forward(wheel_vels);
    odom_msg_.header.set__stamp(time);
    odom_msg_.twist.linear.set__x(odom[0]);
    odom_msg_.twist.linear.set__y(odom[1]);
    odom_msg_.twist.linear.set__z(0.0);
    odom_msg_.twist.angular.set__x(0.0);
    odom_msg_.twist.angular.set__y(0.0);
    odom_msg_.twist.angular.set__z(odom[2]);
    odom_pub_->publish(odom_msg_);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Command writing
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::write_wheel_commands()
{
    auto wheel_vels = wheel_ik_->inverse(base_vel_[0], base_vel_[1], base_vel_[2]);

    for (size_t g = 0; g < wheel_groups_.size(); g++)
    {
        for (const auto & jnt : wheel_groups_[g])
        {
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, wheel_vels[g]);
            if (!sim_flag_)
            {
                set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, std::nan("1"));
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
                set_command(jnt + "/" + hw_if::KP_SCALE, 0.0);
                set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
            }
        }
    }
}

void OmniController::write_leg_commands()
{
    for (const auto & jnt : leg_joints_)
    {
        set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, leg_pos_cmd_[jnt]);
        set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, leg_vel_cmd_[jnt]);
        set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT,   leg_eff_cmd_[jnt]);
        if (!sim_flag_)
        {
            set_command(jnt + "/" + hw_if::KP_SCALE, leg_kp_cmd_[jnt]);
            set_command(jnt + "/" + hw_if::KD_SCALE, leg_kd_cmd_[jnt]);
        }
    }
}

void OmniController::zero_all_commands()
{
    for (const auto & jnt : all_motor_joints_)
    {
        set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
        if (!sim_flag_)
        {
            set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, 0.0);
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
            set_command(jnt + "/" + hw_if::KP_SCALE, 0.0);
            set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Homing
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::homing_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (c_stt_ == ControllerState::INACTIVE && has_homing_ && req->data)
    {
        homing_phase_ = 0;
        homing_time_initialized_ = false;
        c_stt_ = ControllerState::HOMING;
        res->success = true;
        res->message = "Homing started";
        RCLCPP_INFO(get_node()->get_logger(), "Homing started");
    }
    else
    {
        res->success = false;
        res->message = req->data ? "Cannot start homing (not INACTIVE or no homing config)"
                                 : "Invalid request";
    }
}

double OmniController::cosine_interp(double a, double b, double t)
{
    return a + 0.5 * (1.0 - std::cos(M_PI * t)) * (b - a);
}

void OmniController::update_homing(const rclcpp::Time & time)
{
    // Initialize timer on first call of each phase
    if (!homing_time_initialized_)
    {
        homing_start_time_ = time;
        homing_time_initialized_ = true;

        // On phase 0 start, read current joint positions as the actual origin
        if (homing_phase_ == 0)
        {
            for (const auto & jnt : leg_joints_)
                homing_q_start_[jnt] = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
        }
    }

    double elapsed = (time - homing_start_time_).seconds();
    double duration = homing_phase_durations_[static_cast<size_t>(homing_phase_)];
    double t = std::clamp(elapsed / duration, 0.0, 1.0);

    // Interpolate leg joints
    for (const auto & jnt : leg_joints_)
    {
        const auto & cfg = homing_config_[jnt];
        double q_start = (homing_phase_ == 0) ? homing_q_start_[jnt] : cfg.q1;
        double q_end   = (homing_phase_ == 0) ? cfg.q1 : cfg.q2;
        double q = cosine_interp(q_start, q_end, t);

        set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, q);
        set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
        set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
        if (!sim_flag_)
        {
            set_command(jnt + "/" + hw_if::KP_SCALE, 1.0);
            set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
        }
    }

    // Lock wheels during homing
    if (has_wheels_)
    {
        for (const auto & jnt : wheel_joints_)
        {
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
            if (!sim_flag_)
            {
                set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, std::nan("1"));
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
                set_command(jnt + "/" + hw_if::KP_SCALE, 0.0);
                set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
            }
        }
    }

    // Handle phase transitions (after commands are written)
    if (t >= 1.0)
    {
        if (homing_phase_ == 0)
        {
            homing_phase_ = 1;
            homing_time_initialized_ = false;
            RCLCPP_INFO(get_node()->get_logger(), "Homing phase 0 complete, starting phase 1");
        }
        else
        {
            // Homing complete — set leg buffers to q2 for future ACTIVE use
            for (const auto & jnt : leg_joints_)
            {
                const auto & cfg = homing_config_[jnt];
                leg_pos_cmd_[jnt] = cfg.q2;
                leg_vel_cmd_[jnt] = 0.0;
                leg_eff_cmd_[jnt] = 0.0;
                leg_kp_cmd_[jnt]  = 1.0;
                leg_kd_cmd_[jnt]  = 1.0;
            }
            c_stt_ = ControllerState::INACTIVE;
            RCLCPP_INFO(get_node()->get_logger(), "Homing complete, returning to INACTIVE");
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

double OmniController::get_state(const std::string & key) const
{
    auto it = stt_idx_.find(key);
    if (it != stt_idx_.end())
        return state_interfaces_[it->second].get_value();
    return 0.0;
}

void OmniController::set_command(const std::string & key, double value)
{
    auto it = cmd_idx_.find(key);
    if (it != cmd_idx_.end())
        command_interfaces_[it->second].set_value(value);
}

}  // namespace omni_controller

PLUGINLIB_EXPORT_CLASS(
    omni_controller::OmniController, controller_interface::ControllerInterface
)
