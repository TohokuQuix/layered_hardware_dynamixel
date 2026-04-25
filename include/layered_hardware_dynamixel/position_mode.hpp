#ifndef LAYERED_HARDWARE_DYNAMIXEL_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_POSITION_MODE_HPP

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

class PositionMode : public OperatingModeInterface {
public:
  PositionMode(const std::shared_ptr<DynamixelActuatorContext> &context,
               const std::map< std::string, std::int32_t > &item_map)
      : OperatingModeInterface("position", context), item_map_(item_map) {
    for (const auto &[item_name, item_value] : item_map_) {
      if (item_name.rfind("Goal_", 0) == 0) {
        deferred_item_map_.emplace(item_name, item_value);
      } else {
        initial_item_map_.emplace(item_name, item_value);
      }
    }
  }

  virtual void starting() override {
    // For DYNAMIXEL-Y, Goal values cannot be written while Controller State is
    // Process Torque On/Off, and Goal updates begin after Goal Update Delay.
    // Apply non-goal configuration first, enable torque, wait until Goal writes
    // are accepted, then write and confirm deferred Goal_* values.
    if (!set_operating_mode_with_torque_off(context_, &DynamixelWorkbench::setPositionControlMode, 3)) {
      throw std::runtime_error("PositionMode::starting(): Failed to enable operating mode for " +
                               get_display_name(*context_));
    }

    if (!write_items(context_, initial_item_map_)) {
      throw std::runtime_error("PositionMode::starting(): Failed to apply item_map for " +
                               get_display_name(*context_));
    }

    const char *log = nullptr;
    if (!context_->dxl_wb->torqueOn(context_->id, &log)) {
      throw std::runtime_error("PositionMode::starting(): Failed to enable torque for " +
                               get_display_name(*context_) + ": " +
                               (log ? log : "No log from DynamixelWorkbench::torqueOn()"));
    }

    if (!deferred_item_map_.empty() && !wait_until_goal_values_writable(context_)) {
      throw std::runtime_error("PositionMode::starting(): Goal values are not writable for " +
                               get_display_name(*context_));
    }

    for (const auto &[item_name, item_value] : deferred_item_map_) {
      if (!write_item_and_confirm(context_, item_name, item_value)) {
        throw std::runtime_error("PositionMode::starting(): Failed to apply " + item_name +
                                 " for " + get_display_name(*context_));
      }
    }
    log_applied_config(context_, "position");

    // use the present position as the initial command
    read_all_states(context_);
    context_->pos_cmd = context_->pos;
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // read pos, vel, eff, etc
    if (!context_->use_sync_read) {
      read_all_states(context_);
    }
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // write goal position if the goal pos or profile velocity have been updated
    // to make the change affect
    if (!std::isnan(context_->pos_cmd) && context_->pos_cmd != prev_pos_cmd_) {
      if (context_->use_sync_write) {
        if (enqueue_position_command(context_)) {
          prev_pos_cmd_ = context_->pos_cmd;
        }
      } else {
        if (write_position_command(context_)) {
          prev_pos_cmd_ = context_->pos_cmd;
        }
      }
    }
  }

  virtual void stopping() override {
    if (context_->torque_off_on_stop) {
      torque_off(context_);
    }
  }

private:
  const std::map<std::string, std::int32_t> item_map_;
  std::map<std::string, std::int32_t> initial_item_map_;
  std::map<std::string, std::int32_t> deferred_item_map_;
  double prev_pos_cmd_;
};
} // namespace layered_hardware_dynamixel

#endif
