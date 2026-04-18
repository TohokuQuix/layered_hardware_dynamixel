#ifndef LAYERED_HARDWARE_DYNAMIXEL_COMMON_NAMESPACES_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_COMMON_NAMESPACES_HPP

namespace controller_interface {}

namespace hardware_interface {}

namespace layered_hardware {}

namespace layered_hardware_dynamixel {
namespace ci = controller_interface;
namespace hi = hardware_interface;
namespace lh = layered_hardware;
inline constexpr const char *HW_IF_VOLTAGE = "voltage";
inline constexpr const char *HW_IF_LED = "led";
inline constexpr const char *HW_IF_LED_RED = "led_red";
inline constexpr const char *HW_IF_LED_GREEN = "led_green";
inline constexpr const char *HW_IF_LED_BLUE = "led_blue";
} // namespace layered_hardware_dynamixel

#endif
