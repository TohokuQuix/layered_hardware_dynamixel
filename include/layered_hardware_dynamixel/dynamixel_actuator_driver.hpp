#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_DRIVER_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_DRIVER_HPP

#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_dynamixel/clear_multi_turn_mode.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/current_based_position_mode.hpp>
#include <layered_hardware_dynamixel/current_mode.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/extended_position_mode.hpp>
#include <layered_hardware_dynamixel/logging_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <layered_hardware_dynamixel/position_mode.hpp>
#include <layered_hardware_dynamixel/reboot_mode.hpp>
#include <layered_hardware_dynamixel/torque_mode.hpp>
#include <layered_hardware_dynamixel/torque_disable_mode.hpp>
#include <layered_hardware_dynamixel/velocity_mode.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware_dynamixel {

class DynamixelActuatorDriver {
public:
  DynamixelActuatorDriver(const std::string &name, const YAML::Node &params,
                          const std::shared_ptr<DynamixelWorkbench> &dxl_wb,
                          const bool default_torque_off_on_stop) {
    // parse parameters for this actuator
    std::uint8_t id;
    double torque_constant;
    bool torque_off_on_stop = default_torque_off_on_stop;
    std::vector<std::string> mapped_mode_names;
    try {
      id = static_cast<std::uint8_t>(params["id"].as<int>());
      torque_constant = params["torque_constant"].as<double>();
      torque_off_on_stop =
          params["torque_off_on_stop"].as<bool>(default_torque_off_on_stop);
      for (const auto &iface_mode_name_pair : params["operating_mode_map"]) {
        bound_interfaces_.emplace_back(iface_mode_name_pair.first.as<std::string>());
        mapped_mode_names.emplace_back(iface_mode_name_pair.second.as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      throw std::runtime_error("Failed to parse parameters for \"" + name +
                               "\" actuator: " + error.what());
    }

    // allocate context
    context_.reset(
        new DynamixelActuatorContext{name, dxl_wb, id, torque_constant, torque_off_on_stop});
    context_->led_item_name = find_led_item_name(context_);
    context_->led_red_item_name = find_led_red_item_name(context_);
    context_->led_green_item_name = find_led_green_item_name(context_);
    context_->led_blue_item_name = find_led_blue_item_name(context_);

    // find dynamixel actuator by id
    if (!ping(context_)) {
      std::ostringstream msg;
      msg << "Failed to ping " << get_display_name(*context_);
      throw std::runtime_error(msg.str());
    }
    log_startup_config(context_);

    // make operating mode map from ros-controller name to dynamixel's operating mode
    for (const auto &mode_name : mapped_mode_names) {
      std::map<std::string, std::int32_t> item_map;
      get_int32_map_param(params["item_map"], mode_name, item_map);
      try {
        mapped_modes_.emplace_back(make_operating_mode(mode_name, item_map));
      } catch (const std::runtime_error &error) {
        throw std::runtime_error("Invalid value in \"operating_mode_map\" parameter for " +
                                 get_display_name(*context_) + ": " + error.what());
      }
    }
  }

  virtual ~DynamixelActuatorDriver() = default;

  std::vector<hi::StateInterface> export_state_interfaces() {
    // export reference to actuator states owned by this actuator
    std::vector<hi::StateInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff);
    ifaces.emplace_back(context_->name, HW_IF_VOLTAGE, &context_->voltage);
    return ifaces;
  }

  std::vector<hi::CommandInterface> export_command_interfaces() {
    // export reference to actuator commands owned by this actuator
    std::vector<hi::CommandInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff_cmd);
    if (!context_->led_item_name.empty()) {
      ifaces.emplace_back(context_->name, HW_IF_LED, &context_->led_cmd);
    }
    if (!context_->led_red_item_name.empty()) {
      ifaces.emplace_back(context_->name, HW_IF_LED_RED, &context_->led_red_cmd);
    }
    if (!context_->led_green_item_name.empty()) {
      ifaces.emplace_back(context_->name, HW_IF_LED_GREEN, &context_->led_green_cmd);
    }
    if (!context_->led_blue_item_name.empty()) {
      ifaces.emplace_back(context_->name, HW_IF_LED_BLUE, &context_->led_blue_cmd);
    }
    return ifaces;
  }

  hi::return_type prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() <= 1) {
      return hi::return_type::OK;
    } else { // active_bound_ifaces.size() >= 2
      lhd_error("DynamixelActuatorDriver::prepare_command_mode_switch(): "
                "Reject mode switching of %s because %zd bound interfaces are about to be active",
                get_display_name(*context_), active_bound_ifaces.size());
      return hi::return_type::ERROR;
    }
  }

  hi::return_type perform_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() >= 2) {
      lhd_error("DynamixelActuatorDriver::perform_command_mode_switch(): "
                "Could not switch mode of %s because %zd bound interfaces are active",
                get_display_name(*context_), bound_interfaces_.size());
      return hi::return_type::ERROR;
    }

    // switch to actuator command mode associated with active bound interface
    if (!active_bound_ifaces.empty()) { // active_bound_ifaces.size() == 1
      switch_operating_modes(mapped_modes_[active_bound_ifaces.front()]);
    } else { // active_bound_ifaces.size() == 0
      // Preserve the last active actuator mode when controllers deactivate.
      // This avoids dropping servo holding state during ros2_control shutdown.
    }
    return hi::return_type::OK;
  }

  hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    } else {
      // Keep one consistent read path regardless of controller activation.
      // When SyncRead already updated states in layer->read(), don't overwrite
      // with per-servo reads here.
      if (!context_->use_sync_read) {
        if (!read_all_states(context_)) {
          lhd_error("DynamixelActuatorDriver::read(): Failed to read state from %s",
                    get_display_name(*context_));
          return hi::return_type::ERROR;
        }
      }
    }
    return hi::return_type::OK;
  }

  hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (!write_led_command(context_)) {
      lhd_error("DynamixelActuatorDriver::write(): Failed to write LED command to %s",
                get_display_name(*context_));
      return hi::return_type::ERROR;
    }
    if (!write_led_red_command(context_)) {
      lhd_error("DynamixelActuatorDriver::write(): Failed to write LED_RED command to %s",
                get_display_name(*context_));
      return hi::return_type::ERROR;
    }
    if (!write_led_green_command(context_)) {
      lhd_error("DynamixelActuatorDriver::write(): Failed to write LED_GREEN command to %s",
                get_display_name(*context_));
      return hi::return_type::ERROR;
    }
    if (!write_led_blue_command(context_)) {
      lhd_error("DynamixelActuatorDriver::write(): Failed to write LED_BLUE command to %s",
                get_display_name(*context_));
      return hi::return_type::ERROR;
    }
    if (present_mode_) {
      present_mode_->write(time, period);
    }
    return hi::return_type::OK; // TODO: return result of write
  }

  const std::shared_ptr<DynamixelActuatorContext> &get_context() const { return context_; }

private:
  static bool get_int32_map_param(const YAML::Node &node,
                                  const std::string &key,
                                  std::map<std::string, std::int32_t> &item_map) {
    try {
      if (!node || !node[key]) {
        lhd_info("get_int32_map_param(): Parameter \"%s\" not found. passing..", key);
        return false;
      }
      for (const auto &item : node[key]) {
        item_map[item.first.as<std::string>()] = item.second.as<std::int32_t>();
      }
    } catch (const YAML::Exception &error) {
      lhd_error("get_int32_map_param(): Failed to parse parameter: %s", error.what());
      return false;
    }
    return true;
  }

  std::shared_ptr<OperatingModeInterface> make_operating_mode(const std::string &mode_str, const std::map<std::string, std::int32_t> &item_map) const {
    if (mode_str == "clear_multi_turn") {
      return std::make_shared<ClearMultiTurnMode>(context_);
    } else if (mode_str == "current") {
      return std::make_shared<CurrentMode>(context_, item_map);
    } else if (mode_str == "current_based_position") {
      return std::make_shared<CurrentBasedPositionMode>(context_, item_map);
    } else if (mode_str == "extended_position") {
      return std::make_shared<ExtendedPositionMode>(context_, item_map);
    } else if (mode_str == "position") {
      return std::make_shared<PositionMode>(context_, item_map);
    } else if (mode_str == "reboot") {
      return std::make_shared<RebootMode>(context_);
    } else if (mode_str == "torque") {
      return std::make_shared<TorqueMode>(context_, item_map);
    } else if (mode_str == "torque_disable") {
      return std::make_shared<TorqueDisableMode>(context_);
    } else if (mode_str == "velocity") {
      return std::make_shared<VelocityMode>(context_, item_map);
    } else {
      throw std::runtime_error("Unknown operating mode name \"" + mode_str + "\" for " +
                               get_display_name(*context_));
    }
  }

  void switch_operating_modes(const std::shared_ptr<OperatingModeInterface> &new_mode) {
    // do nothing if no mode switch is requested
    if (present_mode_ == new_mode) {
      return;
    }
    // stop present mode
    if (present_mode_) {
      lhd_info("DynamixelActuatorDriver::switch_operating_modes(): "
               "Stopping \"%s\" operating mode for %s",
               present_mode_->get_name(), get_display_name(*context_));
      present_mode_->stopping();
      present_mode_.reset();
    }
    // start new mode
    if (new_mode) {
      lhd_info("DynamixelActuatorDriver::switch_operating_modes(): "
               "Starting \"%s\" operating mode for %s",
               new_mode->get_name(), get_display_name(*context_));
      new_mode->starting();
      present_mode_ = new_mode;
    }
  }

private:
  std::shared_ptr<DynamixelActuatorContext> context_;

  // present operating mode
  std::shared_ptr<OperatingModeInterface> present_mode_;
  // map from command interface to operating mode
  // (i.e. mapped_modes_[i] is associated with bound_interfaces_[i])
  std::vector<std::string> bound_interfaces_;
  std::vector<std::shared_ptr<OperatingModeInterface>> mapped_modes_;
};

} // namespace layered_hardware_dynamixel

#endif
