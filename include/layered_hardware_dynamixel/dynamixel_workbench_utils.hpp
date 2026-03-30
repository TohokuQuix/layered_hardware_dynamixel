#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_WORKBENCH_UTILS_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_WORKBENCH_UTILS_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/logging_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

// adapter functions to wrap DynamixelWorkbench and provide a more user-friendly interface
//   - display error messages
//   - argument type conversion
//   - workarounds for bugs

// instruction functions

static inline bool ping(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->ping(context->id, &log)) {
    lhd_error("ping(): Failed to ping to %s: %s", //
              get_display_name(*context), (log ? log : "No log from DynamixelWorkbench::ping()"));
    return false;
  }
  return true;
}

static inline bool ping_for(const std::shared_ptr<DynamixelActuatorContext> &context,
                            const rclcpp::Duration &timeout) {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  const rclcpp::Time timeout_abs = clock.now() + timeout;
  while (true) {
    if (clock.now() > timeout_abs) {
      lhd_error("ping_for(): No ping response from %s for %f s", //
                get_display_name(*context), timeout.seconds());
      return false;
    }
    if (ping(context)) {
      return true;
    }
  }
  // never reach here
}

static inline bool reboot(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->reboot(context->id, &log)) {
    lhd_error("reboot(): Failed to reboot %s: %s", //
              get_display_name(*context), (log ? log : "No log from DynamixelWorkbench::reboot()"));
    return false;
  }
  return true;
}

// read functions

static inline bool has_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                            const std::string &item) {
  const char *log = nullptr;
  return context->dxl_wb->getItemInfo(context->id, item.c_str(), &log) != NULL;
}

static inline bool read_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                             const std::string &item, std::int32_t *value) {
  const char *log = nullptr;
  if (!context->dxl_wb->itemRead(context->id, item.c_str(), value, &log)) {
    lhd_error("read_item(): Failed to read control table item \"%s\" of %s: %s", //
              item, get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::itemRead()"));
    return false;
  }
  return true;
}

static inline bool try_read_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                                 const std::string &item, std::int32_t *value) {
  if (!has_item(context, item)) {
    return false;
  }
  return read_item(context, item, value);
}

static inline void log_startup_config(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const std::vector<std::string> items = {
      "Torque_Enable",
      "Operating_Mode",
      "Drive_Mode",
      "Bus_Watchdog",
      "PWM_Limit",
      "Current_Limit",
      "Velocity_Limit",
      "Acceleration_Limit",
      "Min_Position_Limit",
      "Max_Position_Limit",
      "Homing_Offset",
      "Profile_Velocity",
      "Profile_Acceleration",
      "Goal_Current",
      "Velocity_I_Gain",
      "Velocity_P_Gain",
      "Position_D_Gain",
      "Position_I_Gain",
      "Position_P_Gain",
      "Feedforward_2nd_Gain",
      "Feedforward_1st_Gain",
  };
  for (const auto &item : items) {
    std::int32_t value = 0;
    if (!try_read_item(context, item, &value)) {
      const auto name = get_display_name(*context);
      lhd_info("startup_config: %s %s=<unavailable>", name.c_str(), item.c_str());
      continue;
    }
    const auto name = get_display_name(*context);
    lhd_info("startup_config: %s %s=%d", name.c_str(), item.c_str(), value);
  }
}

static inline void log_applied_config(const std::shared_ptr<DynamixelActuatorContext> &context,
                                      const std::string &mode_name) {
  const std::vector<std::string> items = {
      "Torque_Enable",
      "Operating_Mode",
      "Drive_Mode",
      "Bus_Watchdog",
      "PWM_Limit",
      "Current_Limit",
      "Velocity_Limit",
      "Acceleration_Limit",
      "Min_Position_Limit",
      "Max_Position_Limit",
      "Homing_Offset",
      "Profile_Velocity",
      "Profile_Acceleration",
      "Goal_Current",
      "Velocity_I_Gain",
      "Velocity_P_Gain",
      "Position_D_Gain",
      "Position_I_Gain",
      "Position_P_Gain",
      "Feedforward_2nd_Gain",
      "Feedforward_1st_Gain",
  };
  for (const auto &item : items) {
    std::int32_t value = 0;
    const auto name = get_display_name(*context);
    if (!try_read_item(context, item, &value)) {
      lhd_info("applied_config[%s]: %s %s=<unavailable>", mode_name.c_str(), name.c_str(),
               item.c_str());
      continue;
    }
    lhd_info("applied_config[%s]: %s %s=%d", mode_name.c_str(), name.c_str(), item.c_str(), value);
  }
}

static inline bool read_position(const std::shared_ptr<DynamixelActuatorContext> &context) {
  float rad;
  const char *log = nullptr;
  if (!context->dxl_wb->getRadian(context->id, &rad, &log)) {
    lhd_error("read_position(): Failed to read position from %s: %s", //
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::getRadian()"));
    return false;
  }
  context->pos = rad;
  return true;
}

static inline bool read_velocity(const std::shared_ptr<DynamixelActuatorContext> &context) {
  std::int32_t value;
  // As of dynamixel_workbench_toolbox v2.0.0,
  // DynamixelWorkbench::getVelocity() reads a wrong item ...
  if (!read_item(context, "Present_Velocity", &value)) {
    return false;
  }
  context->vel = context->dxl_wb->convertValue2Velocity(context->id, value);
  return true;
}

static inline bool has_effort(const std::shared_ptr<DynamixelActuatorContext> &context) {
  return has_item(context, "Present_Current");
}

static inline bool read_effort(const std::shared_ptr<DynamixelActuatorContext> &context) {
  std::int32_t value;
  if (!read_item(context, "Present_Current", &value)) {
    return false;
  }
  // mA -> N*m
  context->eff =
      context->dxl_wb->convertValue2Current(context->id, value) * context->torque_constant / 1000.0;
  return true;
}

static inline bool read_all_states(const std::shared_ptr<DynamixelActuatorContext> &context) {
  // if one fails, "return read_position() && read_velocity() && ..." does not call others.
  // on the other hand, lines below call all anyway to read info as much as possible.
  const bool pos_result = read_position(context);
  const bool vel_result = read_velocity(context);
  const bool eff_result = has_effort(context) ? read_effort(context) : true;
  return pos_result && vel_result && eff_result;
}

// write functions

static inline bool
enable_operating_mode(const std::shared_ptr<DynamixelActuatorContext> &context,
                      bool (DynamixelWorkbench::*const set_func)(std::uint8_t, const char **),
                      const std::int32_t target_operating_mode) {
  std::int32_t operating_mode = -1;
  std::int32_t torque_enable = -1;
  const bool has_mode = read_item(context, "Operating_Mode", &operating_mode);
  const bool has_torque = read_item(context, "Torque_Enable", &torque_enable);

  if (has_mode && has_torque && operating_mode == target_operating_mode) {
    // Skip unnecessary mode switches to avoid transient failures at startup.
    if (torque_enable != 0) {
      return true;
    }
    const char *log = nullptr;
    for (int attempt = 0; attempt < 3; ++attempt) {
      log = nullptr;
      if (context->dxl_wb->torqueOn(context->id, &log)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    lhd_error("enable_operating_mode(): Failed to enable torque of %s in existing mode: %s",
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::torqueOn()"));
    return false;
  }

  const char *log;
  // disable torque to make the actuator ready to change operating modes
  log = nullptr;
  if (!context->dxl_wb->torqueOff(context->id, &log)) {
    lhd_error("enable_operating_mode(): Failed to disable torque of %s: %s",
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::torqueOff()"));
    return false;
  }
  // change operating modes
  log = nullptr;
  if (!(context->dxl_wb.get()->*set_func)(context->id, &log)) {
    lhd_error("enable_operating_mode(): Failed to set operating mode of %s: %s",
              get_display_name(*context), (log ? log : "No log from DynamixelWorkbench"));
    return false;
  }
  // activate new operating mode by enabling torque
  for (int attempt = 0; attempt < 3; ++attempt) {
    log = nullptr;
    if (context->dxl_wb->torqueOn(context->id, &log)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  lhd_error("enable_operating_mode(): Failed to enable torque of %s: %s",
            get_display_name(*context),
            (log ? log : "No log from DynamixelWorkbench::torqueOn()"));
  return false;
}

static inline bool torque_off(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->torqueOff(context->id, &log)) {
    lhd_error("torque_off(): Failed to disable torque of %s: %s", //
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::torqueOff()"));
    return false;
  }
  return true;
}

static inline bool clear_multi_turn(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->clearMultiTurn(context->id, &log)) {
    lhd_error("clear_multi_turn(): Failed to clear multi turn count of %s: %s",
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::clearMultiTurn()"));
    return false;
  }
  return true;
}

static inline bool write_item(const std::shared_ptr<DynamixelActuatorContext> &context,
                              const std::string &item, const std::int32_t value) {
  const char *log = nullptr;
  if (!context->dxl_wb->itemWrite(context->id, item.c_str(), value, &log)) {
    lhd_error("write_item(): Failed to set control table item \"%s\" of %s: %s", //
              item, get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::itemWrite()"));
    return false;
  }
  return true;
}

static inline bool write_items(
    const std::shared_ptr<DynamixelActuatorContext> &context,
    const std::map<std::string, std::int32_t> &item_map) {
  for (const auto &item : item_map) {
    if (!write_item(context, item.first, item.second)) {
      return false;
    }
  }
  return true;
}

static inline bool
write_position_command(const std::shared_ptr<DynamixelActuatorContext> &context) {
  if (!std::isfinite(context->pos_cmd)) {
    lhd_error("write_position_command(): Invalid goal position command for %s: pos_cmd=%f",
              get_display_name(*context), context->pos_cmd);
    return false;
  }
  const char *log = nullptr;
  if (!context->dxl_wb->goalPosition(context->id, static_cast<float>(context->pos_cmd), &log)) {
    lhd_error("write_position_command(): Failed to set goal position of %s: pos_cmd=%f pos=%f (%s)",
              get_display_name(*context), context->pos_cmd, context->pos,
              (log ? log : "No log from DynamixelWorkbench::goalPosition()"));
    return false;
  }

  const auto now_tp = std::chrono::steady_clock::now();
  context->last_pos_cmd_written = context->pos_cmd;
  context->last_pos_cmd_write_tp = now_tp;
  context->has_last_pos_cmd_write = true;
  context->pos_cmd_pending = false;

  return true;
}

static inline bool
enqueue_position_command(const std::shared_ptr<DynamixelActuatorContext> &context) {
  if (!std::isfinite(context->pos_cmd)) {
    lhd_error("enqueue_position_command(): Invalid goal position command for %s: pos_cmd=%f",
              get_display_name(*context), context->pos_cmd);
    return false;
  }
  context->pos_cmd_pending = true;
  return true;
}

static inline bool
write_velocity_command(const std::shared_ptr<DynamixelActuatorContext> &context) {
  const char *log = nullptr;
  if (!context->dxl_wb->goalVelocity(context->id, static_cast<float>(context->vel_cmd), &log)) {
    lhd_error("write_velocity_command(): Failed to set goal velocity of %s: %s",
              get_display_name(*context),
              (log ? log : "No log from DynamixelWorkbench::goalVelocity()"));
    return false;
  }
  return true;
}

static inline bool
write_profile_velocity(const std::shared_ptr<DynamixelActuatorContext> &context) {
  return write_item(context, "Profile_Velocity",
                    context->dxl_wb->convertVelocity2Value(
                        context->id, static_cast<float>(std::abs(context->vel_cmd))));
}

static inline bool write_effort_command(const std::shared_ptr<DynamixelActuatorContext> &context) {
  // N*m -> mA
  return write_item(
      context, "Goal_Current",
      context->dxl_wb->convertCurrent2Value(
          context->id, static_cast<float>(context->eff_cmd / context->torque_constant * 1000.0)));
}

} // namespace layered_hardware_dynamixel

#endif
