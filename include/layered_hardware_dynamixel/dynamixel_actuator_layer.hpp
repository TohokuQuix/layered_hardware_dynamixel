#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_LAYER_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_LAYER_HPP

#include <memory>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_driver.hpp>
#include <layered_hardware_dynamixel/logging_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware_dynamixel {

class DynamixelActuatorLayer : public lh::LayerInterface {
public:
  virtual ~DynamixelActuatorLayer() override = default;

  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized =
        lh::LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }
    layer_name_ = layer_name;

    // find parameter group for this layer
    const auto params_it = hardware_info.hardware_parameters.find(layer_name);
    if (params_it == hardware_info.hardware_parameters.end()) {
      lhd_error("DynamixelActuatorLayer::on_init(): \"%s\" parameter is missing", layer_name);
      return CallbackReturn::ERROR;
    }

    // parse parameters for this layer as yaml
    std::string serial_iface;
    std::uint32_t baudrate;
    bool torque_off_on_stop = true;
    bool use_sync_read_param = true;
    bool use_sync_write_param = false;
    std::vector<std::string> ator_names;
    std::vector<YAML::Node> ator_params;
    try {
      const YAML::Node params = YAML::Load(params_it->second);
      serial_iface = params["serial_interface"].as<std::string>("/dev/ttyUSB0");
      baudrate = params["baudrate"].as<int>(115200);
      torque_off_on_stop = params["torque_off_on_stop"].as<bool>(true);
      use_sync_read_param = params["use_sync_read"].as<bool>(true);
      use_sync_write_param = params["use_sync_write"].as<bool>(false);
      for (const auto &name_param_pair : params["actuators"]) {
        ator_names.emplace_back(name_param_pair.first.as<std::string>());
        ator_params.emplace_back(name_param_pair.second);
      }
    } catch (const YAML::Exception &error) {
      lhd_error("DynamixelActuatorLayer::on_init(): %s (on parsing \"%s\" parameter)", //
                error, layer_name);
      return CallbackReturn::ERROR;
    }
    if (ator_names.empty()) {
      lhd_error("DynamixelActuatorLayer::on_init(): no actuators configured in \"%s\" parameter",
                layer_name);
      return CallbackReturn::ERROR;
    }

    // open USB serial device
    const auto dxl_wb = std::make_shared<DynamixelWorkbench>();
    if (!dxl_wb->init(serial_iface.c_str(), baudrate)) {
      lhd_error("DynamixelActuatorLayer::on_init(): Failed to open DynamielWorkbench (%s, %d)",
                serial_iface, baudrate);
      return CallbackReturn::ERROR;
    }
    lhd_info("DynamixelActuatorLayer::on_init(): torque_off_on_stop=%s",
             torque_off_on_stop ? "true" : "false");

    // init actuators with param "actuators/<actuator_name>"
    for (std::size_t i = 0; i < ator_names.size(); ++i) {
      try {
        drivers_.emplace_back(
            new DynamixelActuatorDriver(ator_names[i], ator_params[i], dxl_wb, torque_off_on_stop));
      } catch (const std::runtime_error &error) {
        lhd_error("DynamixelActuatorLayer::on_init(): Failed to create driver for \"%s\" actuator",
                  ator_names[i]);
        return CallbackReturn::ERROR;
      }
      lhd_info("DynamixelActuatorLayer::on_init(): Initialized the actuator \"%s\"", ator_names[i]);
      const auto &context = drivers_.back()->get_context();
      context->layer_name = layer_name_;
      contexts_.push_back(context);
      ids_.push_back(context->id);
    }

    if (!drivers_.empty()) {
      sync_read_enabled_ = use_sync_read_param ? init_sync_read(*dxl_wb) : false;
      for (const auto &context : contexts_) {
        context->use_sync_read = sync_read_enabled_;
      }
      if (sync_read_enabled_) {
        lhd_info("DynamixelActuatorLayer::on_init(): SyncRead enabled for %zu actuators",
                 contexts_.size());
      } else {
        lhd_info("DynamixelActuatorLayer::on_init(): SyncRead disabled, using legacy itemRead");
      }

      sync_write_enabled_ = use_sync_write_param ? init_bulk_write(*dxl_wb) : false;
      for (const auto &context : contexts_) {
        context->use_sync_write = sync_write_enabled_;
      }
      if (sync_write_enabled_) {
        lhd_info("DynamixelActuatorLayer::on_init(): BulkWrite enabled for Goal_Position (%zu actuators)",
                 contexts_.size());
      } else {
        lhd_info("DynamixelActuatorLayer::on_init(): BulkWrite disabled, using per-servo goalPosition");
      }
    }

    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    // export reference to actuator states owned by this layer
    std::vector<hi::StateInterface> ifaces;
    for (const auto &driver : drivers_) {
      ifaces = lh::merge(std::move(ifaces), driver->export_state_interfaces());
    }
    return ifaces;
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    // export reference to actuator commands owned by this layer
    std::vector<hi::CommandInterface> ifaces;
    for (const auto &driver : drivers_) {
      ifaces = lh::merge(std::move(ifaces), driver->export_command_interfaces());
    }
    return ifaces;
  }

  virtual ci::InterfaceConfiguration state_interface_configuration() const override {
    // any state interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // any command interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> && /*state_interfaces*/,
                    std::vector<hi::LoanedCommandInterface> && /*command_interfaces*/) override {
    // any interfaces has to be imported from other layers because this layer is "source"
  }

  virtual hi::return_type
  prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->prepare_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type
  perform_command_mode_switch(const lh::StringRegistry &active_interfaces) override {
    // notify controller switching to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->perform_command_mode_switch(active_interfaces));
    }
    return result;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // read from all actuators (prefer SyncRead and fallback to legacy itemRead)
    hi::return_type result = hi::return_type::OK;
    bool use_sync = false;
    if (sync_read_enabled_) {
      use_sync = sync_read_states(period);
      if (!use_sync) {
        lhd_error("DynamixelActuatorLayer::read(): SyncRead failed, falling back to legacy itemRead");
      }
    }

    for (const auto &context : contexts_) {
      context->use_sync_read = use_sync;
    }

    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->read(time, period));
    }

    if (sync_read_enabled_) {
      for (const auto &context : contexts_) {
        context->use_sync_read = true;
      }
    }

    return result;
  }

  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // write to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &driver : drivers_) {
      result = lh::merge(result, driver->write(time, period));
    }
    if (!flush_bulk_write_position()) {
      result = lh::merge(result, hi::return_type::ERROR);
    }
    return result;
  }

private:
  bool init_bulk_write(DynamixelWorkbench &dxl_wb) {
    const char *log = nullptr;
    if (!dxl_wb.initBulkWrite(&log)) {
      lhd_error("DynamixelActuatorLayer::init_bulk_write(): Failed to initialize BulkWrite: %s",
                (log ? log : "No log from DynamixelWorkbench::initBulkWrite()"));
      return false;
    }
    return true;
  }

  bool flush_bulk_write_position() {
    if (!sync_write_enabled_ || contexts_.empty()) {
      return true;
    }

    std::vector<std::shared_ptr<DynamixelActuatorContext>> pending_contexts;
    pending_contexts.reserve(contexts_.size());
    for (const auto &context : contexts_) {
      if (!context->pos_cmd_pending) {
        continue;
      }
      if (!std::isfinite(context->pos_cmd)) {
        lhd_error("flush_bulk_write_position(): Invalid goal position command for %s: pos_cmd=%f",
                  get_display_name(*context), context->pos_cmd);
        context->pos_cmd_pending = false;
        continue;
      }
      pending_contexts.push_back(context);
    }
    if (pending_contexts.empty()) {
      return true;
    }

    const char *log = nullptr;
    auto *dxl_wb = drivers_.front()->get_context()->dxl_wb.get();
    bool add_ok = true;
    for (const auto &context : pending_contexts) {
      const std::int32_t pos_raw =
          context->dxl_wb->convertRadian2Value(context->id, static_cast<float>(context->pos_cmd));
      log = nullptr;
      if (!dxl_wb->addBulkWriteParam(context->id, "Goal_Position", pos_raw, &log)) {
        lhd_error("flush_bulk_write_position(): Failed addBulkWriteParam(Goal_Position) for %s: %s",
                  get_display_name(*context),
                  (log ? log : "No log from DynamixelWorkbench::addBulkWriteParam()"));
        add_ok = false;
        break;
      }
    }

    bool ok = false;
    if (add_ok) {
      log = nullptr;
      ok = dxl_wb->bulkWrite(&log);
    }
    if (ok) {
      const auto now_tp = std::chrono::steady_clock::now();
      for (const auto &context : pending_contexts) {
        context->last_pos_cmd_written = context->pos_cmd;
        context->last_pos_cmd_write_tp = now_tp;
        context->has_last_pos_cmd_write = true;
        context->pos_cmd_pending = false;
      }
      return true;
    }

    lhd_error("flush_bulk_write_position(): Failed bulkWrite(Goal_Position): %s",
              (log ? log : "No log from DynamixelWorkbench::bulkWrite()"));
    bool fallback_ok = true;
    for (const auto &context : pending_contexts) {
      fallback_ok = write_position_command(context) && fallback_ok;
    }
    return fallback_ok;
  }

private:
  bool init_sync_read(DynamixelWorkbench &dxl_wb) {
    if (ids_.empty()) {
      return false;
    }
    if (ids_.size() > std::numeric_limits<std::uint8_t>::max()) {
      lhd_error("DynamixelActuatorLayer::init_sync_read(): actuator count exceeds uint8_t limit");
      return false;
    }

    const char *log = nullptr;
    if (!dxl_wb.addSyncReadHandler(ids_.front(), "Present_Position", &log)) {
      lhd_error("DynamixelActuatorLayer::init_sync_read(): Failed to add Present_Position handler: %s",
                (log ? log : "No log from DynamixelWorkbench::addSyncReadHandler()"));
      return false;
    }
    sync_idx_pos_ = 0;

    log = nullptr;
    if (!dxl_wb.addSyncReadHandler(ids_.front(), "Present_Velocity", &log)) {
      lhd_error("DynamixelActuatorLayer::init_sync_read(): Failed to add Present_Velocity handler: %s",
                (log ? log : "No log from DynamixelWorkbench::addSyncReadHandler()"));
      return false;
    }
    sync_idx_vel_ = 1;

    has_effort_sync_ = true;
    for (const auto &context : contexts_) {
      if (!has_effort(context)) {
        has_effort_sync_ = false;
        break;
      }
    }

    if (has_effort_sync_) {
      log = nullptr;
      if (!dxl_wb.addSyncReadHandler(ids_.front(), "Present_Current", &log)) {
        lhd_error("DynamixelActuatorLayer::init_sync_read(): Failed to add Present_Current handler: %s",
                  (log ? log : "No log from DynamixelWorkbench::addSyncReadHandler()"));
        return false;
      }
      sync_idx_eff_ = 2;
    }

    return true;
  }

  bool sync_read_states(const rclcpp::Duration &period) {
    if (contexts_.empty() || ids_.empty()) {
      return false;
    }

    const auto id_count = static_cast<std::uint8_t>(ids_.size());
    std::vector<std::int32_t> pos_raw(ids_.size());
    std::vector<std::int32_t> vel_raw(ids_.size());
    std::vector<std::int32_t> eff_raw(ids_.size(), 0);

    const char *log = nullptr;
    if (!drivers_.front()->get_context()->dxl_wb->syncRead(sync_idx_pos_, ids_.data(), id_count, &log)) {
      lhd_error("DynamixelActuatorLayer::sync_read_states(): Failed syncRead(Present_Position): %s",
                (log ? log : "No log from DynamixelWorkbench::syncRead()"));
      return false;
    }
    log = nullptr;
    if (!drivers_.front()->get_context()->dxl_wb->getSyncReadData(sync_idx_pos_, ids_.data(), id_count,
                                                                   pos_raw.data(), &log)) {
      lhd_error("DynamixelActuatorLayer::sync_read_states(): Failed getSyncReadData(Present_Position): %s",
                (log ? log : "No log from DynamixelWorkbench::getSyncReadData()"));
      return false;
    }

    log = nullptr;
    if (!drivers_.front()->get_context()->dxl_wb->syncRead(sync_idx_vel_, ids_.data(), id_count, &log)) {
      lhd_error("DynamixelActuatorLayer::sync_read_states(): Failed syncRead(Present_Velocity): %s",
                (log ? log : "No log from DynamixelWorkbench::syncRead()"));
      return false;
    }
    log = nullptr;
    if (!drivers_.front()->get_context()->dxl_wb->getSyncReadData(sync_idx_vel_, ids_.data(), id_count,
                                                                   vel_raw.data(), &log)) {
      lhd_error("DynamixelActuatorLayer::sync_read_states(): Failed getSyncReadData(Present_Velocity): %s",
                (log ? log : "No log from DynamixelWorkbench::getSyncReadData()"));
      return false;
    }

    if (has_effort_sync_) {
      log = nullptr;
      if (!drivers_.front()->get_context()->dxl_wb->syncRead(sync_idx_eff_, ids_.data(), id_count, &log)) {
        lhd_error("DynamixelActuatorLayer::sync_read_states(): Failed syncRead(Present_Current): %s",
                  (log ? log : "No log from DynamixelWorkbench::syncRead()"));
        return false;
      }
      log = nullptr;
      if (!drivers_.front()->get_context()->dxl_wb->getSyncReadData(sync_idx_eff_, ids_.data(), id_count,
                                                                     eff_raw.data(), &log)) {
        lhd_error("DynamixelActuatorLayer::sync_read_states(): Failed getSyncReadData(Present_Current): %s",
                  (log ? log : "No log from DynamixelWorkbench::getSyncReadData()"));
        return false;
      }
    }

    for (std::size_t i = 0; i < contexts_.size(); ++i) {
      const auto &context = contexts_[i];
      // convertValue2Radian() can disagree with getRadian() on some models/settings.
      // Keep position readout consistent with the non-SyncRead path to avoid state jumps.
      if (!read_position(context)) {
        context->pos = context->dxl_wb->convertValue2Radian(context->id, pos_raw[i]);
      }
      context->vel = context->dxl_wb->convertValue2Velocity(context->id, vel_raw[i]);
      // Some models return corrupted velocity in SyncRead intermittently.
      // Fallback to per-servo itemRead when the converted value is clearly non-physical.
      // If fallback read fails, keep the original SyncRead-derived value (packet data) as-is.
      if (!std::isfinite(context->vel) || std::abs(context->vel) > 200.0) {
        (void)read_velocity(context);
      }
      if (has_effort_sync_) {
        context->eff = context->dxl_wb->convertValue2Current(context->id, static_cast<std::int16_t>(eff_raw[i])) *
                       context->torque_constant / 1000.0;
      }
    }

    return true;
  }

private:
  std::vector<std::unique_ptr<DynamixelActuatorDriver>> drivers_;
  std::vector<std::shared_ptr<DynamixelActuatorContext>> contexts_;
  std::vector<std::uint8_t> ids_;
  bool sync_read_enabled_ = false;
  bool sync_write_enabled_ = false;
  std::string layer_name_ = "unknown";
  bool has_effort_sync_ = false;
  std::uint8_t sync_idx_pos_ = 0;
  std::uint8_t sync_idx_vel_ = 1;
  std::uint8_t sync_idx_eff_ = 2;
};
} // namespace layered_hardware_dynamixel

#endif
