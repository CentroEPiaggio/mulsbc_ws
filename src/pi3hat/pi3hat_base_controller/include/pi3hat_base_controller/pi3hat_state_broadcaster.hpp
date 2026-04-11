#ifndef PI3HAT_STATE_BROADCASTER_HPP
#define PI3HAT_STATE_BROADCASTER_HPP
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"


namespace pi3hat_state_broadcaster {
using LossMsgs = pi3hat_moteus_int_msgs::msg::PacketPass;
using StateMsgs = pi3hat_moteus_int_msgs::msg::JointsStates;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class Pi3HatStateBroadcaster: public controller_interface::ControllerInterface {
public:
    Pi3HatStateBroadcaster();

    ~Pi3HatStateBroadcaster() {};

    CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type
    update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // probably joints_ is useless
    std::vector<std::string> joints_;
    std::vector<bool> se_flag_;
    std::string logger_name_;
    LossMsgs per_msg_;
    StateMsgs stt_msg_;
    rclcpp::Publisher<LossMsgs>::SharedPtr per_pub_;
    rclcpp::Publisher<StateMsgs>::SharedPtr stt_pub_;
    // Interface indices cache
    std::vector<size_t> position_indices_, velocity_indices_, effort_indices_;
    std::vector<size_t> temperature_indices_, current_indices_;
    // MODIFICA: Aggiunti vettori per voltage, power e package_loss per cache degli indici delle
    // interfacce Per tornare indietro rimuovi queste righe e usa indici hardcoded nel update()
    std::vector<size_t> voltage_indices_, power_indices_, package_loss_indices_;
    std::vector<std::pair<size_t, size_t>> se_pos_vel_indices_; // (position, velocity) per cada SE
};
}; // namespace pi3hat_state_broadcaster


#endif
