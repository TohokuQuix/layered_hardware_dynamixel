#ifndef LAYERED_HARDWARE_DYNAMIXEL_CURRENT_BASED_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CURRENT_BASED_POSITION_MODE_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class CurrentBasedPositionMode : public OperatingModeInterface {
public:
  CurrentBasedPositionMode(const std::shared_ptr<DynamixelActuatorContext> &context,
                           const std::map<std::string, std::int32_t> &item_map)
      : OperatingModeInterface("current_based_position", context), item_map_(item_map) {
    const auto it = item_map_.find("Goal_Current");
    if (it != item_map_.end()) {
      configured_goal_current_ = it->second;
    }
  }

  virtual void starting() override {
    // switch to current-based position mode
    if (!enable_operating_mode(context_, &DynamixelWorkbench::setCurrentBasedPositionControlMode, 5)) {
      throw std::runtime_error(
          "CurrentBasedPositionMode::starting(): Failed to enable operating mode for " +
          get_display_name(*context_));
    }

    write_items(context_, item_map_);
    log_applied_config(context_, "current_based_position");

    // use the present position as the initial position command
    read_all_states(context_);
    context_->pos_cmd = context_->pos;
    context_->vel_cmd = 0.; // use velocity limit in the dynamixel's control table
    // Keep Goal_Current from item_map until an explicit effort command is given.
    context_->eff_cmd = std::numeric_limits<double>::quiet_NaN();
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
    prev_vel_cmd_ = std::numeric_limits<double>::quiet_NaN();
    prev_eff_cmd_ = std::numeric_limits<double>::quiet_NaN();

    cached_pos_ = std::nullopt;
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // read pos, vel, eff, etc
    if (!context_->use_sync_read) {
      read_all_states(context_);
    }
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // write profile velocity if updated
    const bool do_write_vel =
        (!std::isnan(context_->vel_cmd) && context_->vel_cmd != prev_vel_cmd_);
    if (do_write_vel) {
      write_profile_velocity(context_);
      prev_vel_cmd_ = context_->vel_cmd;
    }

    // write effort limit if updated
    const bool do_write_eff =
        (!std::isnan(context_->eff_cmd) && context_->eff_cmd != prev_eff_cmd_);
    if (do_write_eff) {
      write_effort_command(context_);
      prev_eff_cmd_ = context_->eff_cmd;
    }

    // if the profile velocity is 0, the user would want the actuator
    // to stop at the present position but 0 actually means unlimited.
    // to solve this mismatch, freeze the position command on that case.
    const bool do_freeze_pos =
        (!std::isnan(context_->vel_cmd) &&
         context_->dxl_wb->convertVelocity2Value(context_->id, context_->vel_cmd) == 0);
    if (do_freeze_pos) {
      if (!cached_pos_) {
        cached_pos_ = context_->pos;
      }
      context_->pos_cmd = cached_pos_.value();
    } else {
      cached_pos_ = std::nullopt;
    }

    // write goal position if the goal pos, profile velocity or effort limit have been updated
    // to make the change affect
    const bool do_write_pos =
        (!std::isnan(context_->pos_cmd) &&
         (do_write_vel || do_write_eff || context_->pos_cmd != prev_pos_cmd_));
    if (do_write_pos) {
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

    // Some setups appear to restore Goal_Current after startup/position updates.
    // Keep the configured clamp unless an explicit effort command is being used.
    if (configured_goal_current_ && std::isnan(context_->eff_cmd)) {
      write_item(context_, "Goal_Current", configured_goal_current_.value());
    }
  }

  virtual void stopping() override {
    if (context_->torque_off_on_stop) {
      torque_off(context_);
    }
  }

private:
  const std::map<std::string, std::int32_t> item_map_;
  double prev_pos_cmd_, prev_vel_cmd_, prev_eff_cmd_;
  std::optional<double> cached_pos_;
  std::optional<std::int32_t> configured_goal_current_;
};
} // namespace layered_hardware_dynamixel

#endif
