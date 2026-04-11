#include "pi3hat_base_controller/pi3hat_state_broadcaster.hpp"
#include "pi3hat_hw_interface/actuator_manager.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <cstdint>

namespace pi3hat_state_broadcaster {
Pi3HatStateBroadcaster::Pi3HatStateBroadcaster()
: logger_name_("Pi3HatStateBroadcaster"),
  per_pub_(nullptr),
  stt_pub_(nullptr) {};

CallbackReturn Pi3HatStateBroadcaster::on_init()
{

    try {
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("second_encoders", std::vector<std::string>());
        auto_declare<bool>("performance_index", true);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(logger_name_),
            "Exception thrown during declaration of joints name with message: %s", e.what()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "initialize succesfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn Pi3HatStateBroadcaster::on_configure(const rclcpp_lifecycle::State&)
{
    bool per_ind, err_se_name = false, se_prov;
    std::vector<std::string> sec_enc;
    joints_ = get_node()->get_parameter("joints").as_string_array();
    if (joints_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "'joints' parameter is empty");
        return CallbackReturn::ERROR;
    }
    se_flag_.resize(joints_.size(), false);
    sec_enc = get_node()->get_parameter("second_encoders").as_string_array();

    if (!sec_enc.empty()) {
        for (auto& se_name : sec_enc) {
            err_se_name = true;
            for (size_t i = 0; i < joints_.size(); i++) {
                if (joints_[i] == se_name) {
                    err_se_name = false;
                    se_flag_[i] = true;
                }
            }
            if (err_se_name) {
                RCLCPP_ERROR(
                    rclcpp::get_logger(logger_name_),
                    "'second_encoder' named %s are not contained into 'joints' ", se_name.c_str()
                );
                return CallbackReturn::ERROR;
            }
        }
        se_prov = true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger(logger_name_), "'second_encoder' is empty");
        se_prov = false;
    }
    per_ind = get_node()->get_parameter("performance_index").as_bool();

    stt_msg_.name.resize(joints_.size());
    stt_msg_.position.resize(joints_.size());
    stt_msg_.velocity.resize(joints_.size());
    stt_msg_.effort.resize(joints_.size());
    stt_msg_.temperature.resize(joints_.size());
    stt_msg_.current.resize(joints_.size());
    // MODIFICA: Aggiunto resize per power e voltage nel messaggio
    // Per rimuovere power e voltage, commenta o rimuovi queste due righe
    stt_msg_.power.resize(
        joints_.size()
    ); // COMMENTO: Rimuovi questa riga se non vuoi leggere power dal motore
    stt_msg_.voltage.resize(
        joints_.size()
    ); // COMMENTO: Rimuovi questa riga se non vuoi leggere voltage dal motore
    if (se_prov) {
        stt_msg_.sec_enc_pos.resize(joints_.size());
        stt_msg_.sec_enc_vel.resize(joints_.size());
    }

    stt_pub_ = get_node()->create_publisher<StateMsgs>("~/joints_state", 5);

    if (per_ind) {
        per_msg_.name.resize(joints_.size());
        per_msg_.pack_loss.resize(joints_.size());
        for (size_t i = 0; i < joints_.size(); i++)
            per_msg_.name[i] = joints_[i];

        per_pub_ = get_node()->create_publisher<LossMsgs>("~/performance_indexes", 5);
    }
    RCLCPP_INFO(get_node()->get_logger(), "configurated succesfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn Pi3HatStateBroadcaster::on_cleanup(const rclcpp_lifecycle::State&)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn Pi3HatStateBroadcaster::on_activate(const rclcpp_lifecycle::State&)
{
    // MODIFICA: Sostituito l'uso di indici hardcoded con cache degli indici basata sui nomi delle
    // interfacce Questo rende il codice robusto all'ordine delle interfacce esportate dall'hardware
    // interface Nel codice originale, si assumeva un ordine fisso: position, velocity, effort,
    // q_current, power, voltage, temperature, package_loss Ma l'ordine poteva variare, causando
    // letture errate (es. power letto come zero) Per tornare alla versione originale con indici
    // hardcoded, commenta tutto questo blocco e usa: position_indices_[i] = 2 + i*8;
    // velocity_indices_[i] = 3 + i*8; etc. Ma attenzione: l'ordine deve corrispondere a quello
    // esportato dall'hardware (vedi actuator_manager.cpp ExportSttInt)
    position_indices_.resize(joints_.size());
    velocity_indices_.resize(joints_.size());
    effort_indices_.resize(joints_.size());
    temperature_indices_.resize(joints_.size());
    current_indices_.resize(joints_.size());
    voltage_indices_.resize(joints_.size());
    power_indices_.resize(joints_.size());
    package_loss_indices_.resize(joints_.size());

    for (size_t i = 0; i < joints_.size(); i++) {
        // Find index for each interface by name
        for (size_t j = 0; j < state_interfaces_.size(); j++) {
            auto name = state_interfaces_[j].get_name();
            if (name == joints_[i] + "/" + hardware_interface::HW_IF_POSITION)
                position_indices_[i] = j;
            else if (name == joints_[i] + "/" + hardware_interface::HW_IF_VELOCITY)
                velocity_indices_[i] = j;
            else if (name == joints_[i] + "/" + hardware_interface::HW_IF_EFFORT)
                effort_indices_[i] = j;
            else if (name == joints_[i] + "/" + hardware_interface::HW_IF_TEMPERATURE)
                temperature_indices_[i] = j;
            else if (name == joints_[i] + "/" + hardware_interface::HW_IF_Q_CURRENT)
                current_indices_[i] = j;
            else if (name == joints_[i] + "/" + hardware_interface::HW_IF_VOLTAGE)
                voltage_indices_[i] = j;
            else if (name == joints_[i] + "/" + hardware_interface::HW_IF_POWER)
                power_indices_[i] = j;
            else if (name == joints_[i] + "/" + hardware_interface::HW_IF_PACKAGE_LOSS)
                package_loss_indices_[i] = j;
        }
    }

    // Cache second encoder indices
    se_pos_vel_indices_.clear();
    for (size_t i = 0; i < joints_.size(); i++) {
        if (se_flag_[i]) {
            size_t pos_idx = 0, vel_idx = 0;
            for (size_t j = 0; j < state_interfaces_.size(); j++) {
                auto name = state_interfaces_[j].get_name();
                if (name == joints_[i] + "_second_encoder/" + hardware_interface::HW_IF_POSITION)
                    pos_idx = j;
                else if (name ==
                         joints_[i] + "_second_encoder/" + hardware_interface::HW_IF_VELOCITY)
                    vel_idx = j;
            }
            se_pos_vel_indices_.push_back({pos_idx, vel_idx});
        }
    }

    RCLCPP_INFO(get_node()->get_logger(), "activated succesfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn Pi3HatStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State&)
{
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Pi3HatStateBroadcaster::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration stt_int_cnf;
    stt_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    std::string int_name = "Pi3Hat";
    stt_int_cnf.names.push_back(int_name + "/" + hardware_interface::HW_IF_VALIDITY_LOSS);
    stt_int_cnf.names.push_back(int_name + "/" + hardware_interface::HW_IF_CYCLE_DUR);
    for (size_t i = 0; i < joints_.size(); i++) {
        stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_POSITION);
        stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
        stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_EFFORT);
        stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_Q_CURRENT);
        // MODIFICA: Aggiunte interfacce per power e voltage
        // Per rimuovere power e voltage, commenta o rimuovi queste due righe
        stt_int_cnf.names.push_back(
            joints_[i] + "/" + hardware_interface::HW_IF_POWER
        ); // COMMENTO: Rimuovi questa riga se non vuoi leggere power
        stt_int_cnf.names.push_back(
            joints_[i] + "/" + hardware_interface::HW_IF_VOLTAGE
        ); // COMMENTO: Rimuovi questa riga se non vuoi leggere voltage
        stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_TEMPERATURE);
        stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_PACKAGE_LOSS);
    }
    for (size_t i = 0; i < joints_.size(); i++) {
        if (se_flag_[i]) {
            stt_int_cnf.names.push_back(
                joints_[i] + "_second_encoder/" + hardware_interface::HW_IF_POSITION
            );
            stt_int_cnf.names.push_back(
                joints_[i] + "_second_encoder/" + hardware_interface::HW_IF_VELOCITY
            );
        }
    }
    // RCLCPP_INFO(get_node()->get_logger(),"the dimesion of hw is %ld",stt_int_cnf.names.size());
    return stt_int_cnf;
}

controller_interface::InterfaceConfiguration
Pi3HatStateBroadcaster::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cmd_int_cnf;
    cmd_int_cnf.type = controller_interface::interface_configuration_type::NONE;
    return cmd_int_cnf;
}

controller_interface::return_type
Pi3HatStateBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration&)
{
    size_t sz = joints_.size();
    if (per_pub_ != nullptr) {
        per_msg_.header.set__stamp(time);
        per_msg_.set__valid(state_interfaces_[0].get_value());
        per_msg_.set__cycle_dur(state_interfaces_[1].get_value());
        // RCLCPP_INFO_STREAM(get_node()->get_logger(),state_interfaces_[2].get_value());
        for (size_t i = 0; i < sz; i++) {
            per_msg_.pack_loss[i] = state_interfaces_[package_loss_indices_[i]].get_value();
        }

        per_pub_->publish(per_msg_);
    }


    stt_msg_.header.set__stamp(time);
    for (size_t i = 0; i < sz; i++) {
        stt_msg_.name[i] = joints_[i];
        // MODIFICA: Sostituiti indici hardcoded con quelli dalla cache basata sui nomi
        // Nel codice originale: stt_msg_.position[i] = state_interfaces_[2 + i*8].get_value(); etc.
        // Questo evita errori se l'ordine delle interfacce cambia
        stt_msg_.position[i] = state_interfaces_[position_indices_[i]].get_value();
        stt_msg_.velocity[i] = state_interfaces_[velocity_indices_[i]].get_value();
        stt_msg_.effort[i] = state_interfaces_[effort_indices_[i]].get_value();
        stt_msg_.current[i] = state_interfaces_[current_indices_[i]].get_value();
        // MODIFICA: Aggiunte letture per power e voltage
        // Per rimuovere, commenta queste due righe
        stt_msg_.power[i] = state_interfaces_[power_indices_[i]].get_value();
        stt_msg_.voltage[i] = state_interfaces_[voltage_indices_[i]].get_value();
        stt_msg_.temperature[i] = state_interfaces_[temperature_indices_[i]].get_value();
    }

    size_t se_index = 0;
    for (size_t i = 0; i < sz; i++) {
        if (se_flag_[i]) {
            stt_msg_.sec_enc_pos[i] =
                state_interfaces_[se_pos_vel_indices_[se_index].first].get_value();
            stt_msg_.sec_enc_vel[i] =
                state_interfaces_[se_pos_vel_indices_[se_index].second].get_value();
            se_index++;
        }
    }
    stt_pub_->publish(stt_msg_);

    return controller_interface::return_type::OK;
}
}; // namespace pi3hat_state_broadcaster

PLUGINLIB_EXPORT_CLASS(
    pi3hat_state_broadcaster::Pi3HatStateBroadcaster, controller_interface::ControllerInterface
);
