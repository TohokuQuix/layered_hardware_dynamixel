#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP

#include <cstdint>
#include <chrono>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

namespace layered_hardware_dynamixel {

struct DynamixelActuatorContext {
  DynamixelActuatorContext(const std::string &name,
                           const std::shared_ptr<DynamixelWorkbench> &dxl_wb,
                           const std::uint8_t id,
                           const double torque_constant,
                           const bool torque_off_on_stop)
      : name(name), dxl_wb(dxl_wb), id(id), torque_constant(torque_constant),
        torque_off_on_stop(torque_off_on_stop) {}

  // handles
  const std::string name;
  const std::shared_ptr<DynamixelWorkbench> dxl_wb;
  const std::uint8_t id;

  // params
  const double torque_constant;
  const bool torque_off_on_stop;
  std::string layer_name = "unknown";
  std::string led_item_name;
  std::string led_red_item_name;
  std::string led_green_item_name;
  std::string led_blue_item_name;

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN(),
         voltage = std::numeric_limits<double>::quiet_NaN();
  bool use_sync_read = false;
  bool use_sync_write = false;
  bool pos_cmd_pending = false;

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN(),
         led_cmd = std::numeric_limits<double>::quiet_NaN(),
         led_red_cmd = std::numeric_limits<double>::quiet_NaN(),
         led_green_cmd = std::numeric_limits<double>::quiet_NaN(),
         led_blue_cmd = std::numeric_limits<double>::quiet_NaN();

  // diagnostics (command update timing)
  bool has_last_pos_cmd_write = false;
  double last_pos_cmd_written = std::numeric_limits<double>::quiet_NaN();
  std::chrono::steady_clock::time_point last_pos_cmd_write_tp;
  bool has_last_led_cmd_write = false;
  std::int32_t last_led_cmd_written = 0;
  bool has_last_led_red_cmd_write = false;
  bool has_last_led_green_cmd_write = false;
  bool has_last_led_blue_cmd_write = false;
  std::int32_t last_led_red_cmd_written = 0;
  std::int32_t last_led_green_cmd_written = 0;
  std::int32_t last_led_blue_cmd_written = 0;
};

// utility functions

static inline std::string get_display_name(const DynamixelActuatorContext &context) {
  std::ostringstream disp_name;
  disp_name << "\"" << context.name << "\" actuator (id: " << static_cast<int>(context.id) << ")";
  return disp_name.str();
}

} // namespace layered_hardware_dynamixel

#endif
