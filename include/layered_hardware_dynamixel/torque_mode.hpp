#ifndef LAYERED_HARDWARE_DYNAMIXEL_TORQUE_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_TORQUE_MODE_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class TorqueMode : public OperatingModeInterface {
public:
  TorqueMode(const std::shared_ptr<DynamixelActuatorContext> &context,
             const std::map<std::string, std::int32_t> &item_map)
      : OperatingModeInterface("torque", context), item_map_(item_map) {}

  virtual void starting() override {
    // switch to current mode
    if (!enable_operating_mode(context_, &DynamixelWorkbench::setTorqueControlMode)) {
      throw std::runtime_error("TorqueMode::starting(): Failed to enable operating mode for " +
                               get_display_name(*context_));
    }

    write_items(context_, item_map_);
    log_applied_config(context_, "torque");

    // set reasonable initial command
    context_->eff_cmd = 0.;
    prev_eff_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!context_->use_sync_read) {
      read_all_states(context_);
    }
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->eff_cmd) && context_->eff_cmd != prev_eff_cmd_) {
      write_effort_command(context_);
      prev_eff_cmd_ = context_->eff_cmd;
    }
  }

  virtual void stopping() override {
    if (context_->torque_off_on_stop) {
      torque_off(context_);
    }
  }

private:
  const std::map<std::string, std::int32_t> item_map_;
  double prev_eff_cmd_;
};
} // namespace layered_hardware_dynamixel

#endif
