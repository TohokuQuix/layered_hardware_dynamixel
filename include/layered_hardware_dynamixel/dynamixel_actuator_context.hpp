#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP

#include <cstdint>
#include <chrono>
#include <limits>
#include <sstream>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

namespace layered_hardware_dynamixel {

struct DynamixelActuatorContext {
  // handles
  const std::string name;
  const std::shared_ptr<DynamixelWorkbench> dxl_wb;
  const std::uint8_t id;

  // params
  const double torque_constant;
  const bool torque_off_on_stop;
  std::string layer_name = "unknown";

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN();
  bool use_sync_read = false;
  bool use_sync_write = false;
  bool pos_cmd_pending = false;

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN();

  // diagnostics (command update timing)
  bool has_last_pos_cmd_write = false;
  double last_pos_cmd_written = std::numeric_limits<double>::quiet_NaN();
  std::chrono::steady_clock::time_point last_pos_cmd_write_tp;
};

// utility functions

static inline std::string get_display_name(const DynamixelActuatorContext &context) {
  std::ostringstream disp_name;
  disp_name << "\"" << context.name << "\" actuator (id: " << static_cast<int>(context.id) << ")";
  return disp_name.str();
}

} // namespace layered_hardware_dynamixel

#endif
