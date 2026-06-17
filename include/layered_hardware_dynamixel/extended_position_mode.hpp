#ifndef LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP

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

class ExtendedPositionMode : public OperatingModeInterface {
public:
  ExtendedPositionMode(const std::shared_ptr<DynamixelActuatorContext> &context,
                       const std::map< std::string, std::int32_t > &item_map)
      : OperatingModeInterface("extended_position", context), item_map_(item_map) {}

  virtual void starting() override {
    // switch to extended-position mode & torque enable
    if (!enable_operating_mode(context_, &DynamixelWorkbench::setExtendedPositionControlMode, 4)) {
      const auto msg =
          "ExtendedPositionMode::starting(): Failed to enable operating mode for " +
          get_display_name(*context_);
      lhd_error("STARTUP_FAILURE: %s", msg);
      throw std::runtime_error(msg);
    }

    if (!write_items(context_, item_map_)) {
      const auto msg =
          "ExtendedPositionMode::starting(): Failed to apply item_map for " +
          get_display_name(*context_);
      lhd_error("STARTUP_FAILURE: %s", msg);
      throw std::runtime_error(msg);
    }
    log_applied_config(context_, "extended_position");

    // use the present position as the initial command
    read_all_states(context_);
    context_->pos_cmd = context_->pos;
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
    context_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits<double>::quiet_NaN();

    cached_pos_ = std::nullopt;
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // read pos, vel, eff, etc
    if (!context_->use_sync_read) {
      read_all_states(context_);
    }
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // write goal position if the goal pos has been updated to make the change affect
    const bool do_write_pos =
        (!std::isnan(context_->pos_cmd) && (context_->pos_cmd != prev_pos_cmd_));
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
  }

  virtual void stopping() override {
    if (context_->torque_off_on_stop) {
      torque_off(context_);
    }
  }

private:
  const std::map<std::string, std::int32_t> item_map_;
  double prev_pos_cmd_, prev_vel_cmd_;
  std::optional<double> cached_pos_;
};
} // namespace layered_hardware_dynamixel

#endif
