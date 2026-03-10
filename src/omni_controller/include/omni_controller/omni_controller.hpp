#ifndef OMNI_CONTROLLER_HPP
#define OMNI_CONTROLLER_HPP

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"
#include "pi3hat_moteus_int_msgs/msg/distributors_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "omni_controller/wheel_ik.hpp"

#include <chrono>

namespace omni_controller
{

// Local copies of custom HW interface names (avoids depending on pi3hat_hw_interface)
namespace hw_if
{
    constexpr char KP_SCALE[]       = "kp_scale_value";
    constexpr char KD_SCALE[]       = "kd_scale_value";
    constexpr char TEMPERATURE[]    = "temperature";
    constexpr char Q_CURRENT[]      = "q_current";
    constexpr char VOLTAGE[]        = "voltage";
    constexpr char CURRENT[]        = "current";
    constexpr char VALIDITY_LOSS[]  = "validity_loss";
    constexpr char PACKAGE_LOSS[]   = "package_loss";
    constexpr char CYCLE_DUR[]      = "cycle_duration";
}  // namespace hw_if

enum ControllerState
{
    INACTIVE = 0,
    HOMING,
    ACTIVE,
};

struct JointHomingConfig
{
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using TransactionService = std_srvs::srv::SetBool;
using JointsCommand = pi3hat_moteus_int_msgs::msg::JointsCommand;
using JointsStates  = pi3hat_moteus_int_msgs::msg::JointsStates;
using PacketPass    = pi3hat_moteus_int_msgs::msg::PacketPass;
using DistributorsState = pi3hat_moteus_int_msgs::msg::DistributorsState;

class OmniController : public controller_interface::ControllerInterface
{
public:
    OmniController() = default;
    ~OmniController() override = default;

    CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // ─── Configuration ──────────────────────────────────────────────────
    std::vector<std::string> wheel_joints_;
    std::vector<std::vector<std::string>> wheel_groups_;  // IK index → joint names
    std::vector<std::string> leg_joints_;
    std::vector<std::string> all_motor_joints_;  // wheel_joints_ + leg_joints_
    std::vector<std::string> distributor_names_;
    std::vector<std::string> second_encoder_joints_;
    std::vector<bool> se_flag_;  // per motor joint: has second encoder?

    bool has_wheels_ = false;
    bool has_legs_ = false;
    bool has_distributors_ = false;
    bool has_homing_ = false;
    bool sim_flag_ = false;
    bool pub_odom_ = false;
    bool pub_performance_ = true;

    // ─── Wheel IK ───────────────────────────────────────────────────────
    std::unique_ptr<WheelIK> wheel_ik_;

    // ─── State machine ──────────────────────────────────────────────────
    ControllerState c_stt_ = ControllerState::INACTIVE;
    std::atomic<int> dl_miss_count_{0};

    // ─── Homing ─────────────────────────────────────────────────────────
    std::vector<double> homing_phase_durations_;
    std::map<std::string, JointHomingConfig> homing_config_;
    std::map<std::string, double> homing_q_start_;  // actual joint pos at homing start
    int homing_phase_ = 0;
    rclcpp::Time homing_start_time_;
    bool homing_time_initialized_ = false;

    // ─── Buffered commands (protected by mutex) ─────────────────────────
    std::mutex var_mutex_;
    double base_vel_[3] = {0.0, 0.0, 0.0};

    // Leg commands: per-joint buffered values
    std::map<std::string, double> leg_pos_cmd_;
    std::map<std::string, double> leg_vel_cmd_;
    std::map<std::string, double> leg_eff_cmd_;
    std::map<std::string, double> leg_kp_cmd_;
    std::map<std::string, double> leg_kd_cmd_;

    // ─── Interface index maps ───────────────────────────────────────────
    // Built in on_activate() by iterating command/state_interfaces_
    // Key: "joint_name/interface_name"
    std::map<std::string, size_t> cmd_idx_;
    std::map<std::string, size_t> stt_idx_;

    // ─── Publishers ─────────────────────────────────────────────────────
    rclcpp::Publisher<JointsStates>::SharedPtr stt_pub_;
    rclcpp::Publisher<PacketPass>::SharedPtr per_pub_;
    rclcpp::Publisher<DistributorsState>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odom_pub_;

    // Pre-allocated messages
    JointsStates stt_msg_;
    PacketPass per_msg_;
    DistributorsState dist_msg_;
    geometry_msgs::msg::TwistStamped odom_msg_;

    // ─── Subscribers ────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<JointsCommand>::SharedPtr legs_sub_;

    // ─── Services ───────────────────────────────────────────────────────
    rclcpp::Service<TransactionService>::SharedPtr activate_srv_;
    rclcpp::Service<TransactionService>::SharedPtr emergency_srv_;
    rclcpp::Service<TransactionService>::SharedPtr homing_srv_;

    // ─── Callbacks ──────────────────────────────────────────────────────
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void legs_callback(const JointsCommand::SharedPtr msg);
    void activate_service_cb(const TransactionService::Request::SharedPtr req,
                             const TransactionService::Response::SharedPtr res);
    void emergency_service_cb(const TransactionService::Request::SharedPtr req,
                              const TransactionService::Response::SharedPtr res);
    void homing_service_cb(const TransactionService::Request::SharedPtr req,
                           const TransactionService::Response::SharedPtr res);

    // ─── Update helpers ─────────────────────────────────────────────────
    void publish_joint_states(const rclcpp::Time & time);
    void publish_performance(const rclcpp::Time & time);
    void publish_distributor_states(const rclcpp::Time & time);
    void publish_odometry(const rclcpp::Time & time);
    void write_wheel_commands();
    void write_leg_commands();
    void zero_all_commands();
    void update_homing(const rclcpp::Time & time);
    static double cosine_interp(double a, double b, double t);

    // ─── Helpers ────────────────────────────────────────────────────────
    /// Get state interface value by "joint/interface" key. Returns 0.0 if not found.
    double get_state(const std::string & key) const;
    /// Set command interface value by "joint/interface" key.
    void set_command(const std::string & key, double value);
};

}  // namespace omni_controller

#endif  // OMNI_CONTROLLER_HPP
