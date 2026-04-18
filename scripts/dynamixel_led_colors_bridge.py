#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from layered_hardware_dynamixel.msg import DynamixelLedColors


class DynamixelLedColorsBridge(Node):
    def __init__(self) -> None:
        super().__init__("dynamixel_led_colors_bridge")

        self.declare_parameter("input_topic", "/dynamixel_led_colors")
        self.declare_parameter("red_topic", "/dynamixel_led_red_controller/commands")
        self.declare_parameter("green_topic", "/dynamixel_led_green_controller/commands")
        self.declare_parameter("blue_topic", "/dynamixel_led_blue_controller/commands")
        self.declare_parameter("startup_color_enabled", True)
        self.declare_parameter("startup_red", 0)
        self.declare_parameter("startup_green", 255)
        self.declare_parameter("startup_blue", 0)
        self.declare_parameter("startup_delay_sec", 1.0)
        self.declare_parameter("startup_repeat_count", 10)
        self.declare_parameter(
            "actuator_names",
            [
                "chassis/fr_flipper_joint_actuator",
                "chassis/fl_flipper_joint_actuator",
                "chassis/rl_flipper_joint_actuator",
                "chassis/rr_flipper_joint_actuator",
                "arm/joint1_actuator",
                "arm/joint2_actuator",
                "arm/joint3_actuator",
                "arm/joint4_actuator",
                "arm/joint5_actuator",
                "arm/joint6_actuator",
                "arm/grasp_left_actuator",
                "arm/grasp_right_actuator",
            ],
        )

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        red_topic = self.get_parameter("red_topic").get_parameter_value().string_value
        green_topic = self.get_parameter("green_topic").get_parameter_value().string_value
        blue_topic = self.get_parameter("blue_topic").get_parameter_value().string_value
        startup_color_enabled = (
            self.get_parameter("startup_color_enabled").get_parameter_value().bool_value
        )
        startup_red = self.get_parameter("startup_red").get_parameter_value().integer_value
        startup_green = self.get_parameter("startup_green").get_parameter_value().integer_value
        startup_blue = self.get_parameter("startup_blue").get_parameter_value().integer_value
        startup_delay_sec = (
            self.get_parameter("startup_delay_sec").get_parameter_value().double_value
        )
        startup_repeat_count = (
            self.get_parameter("startup_repeat_count").get_parameter_value().integer_value
        )
        self._actuator_names = list(
            self.get_parameter("actuator_names").get_parameter_value().string_array_value
        )
        self._actuator_indices = {
            actuator_name: index for index, actuator_name in enumerate(self._actuator_names)
        }
        self._startup_publish_remaining = max(0, startup_repeat_count)
        self._startup_color_active = startup_color_enabled

        self._red_values = [0.0] * len(self._actuator_names)
        self._green_values = [0.0] * len(self._actuator_names)
        self._blue_values = [0.0] * len(self._actuator_names)

        self._red_pub = self.create_publisher(Float64MultiArray, red_topic, 10)
        self._green_pub = self.create_publisher(Float64MultiArray, green_topic, 10)
        self._blue_pub = self.create_publisher(Float64MultiArray, blue_topic, 10)
        self._sub = self.create_subscription(
            DynamixelLedColors,
            input_topic,
            self._on_colors,
            10,
        )

        self.get_logger().info(
            f"Bridging {input_topic} for {len(self._actuator_names)} Dynamixel actuators"
        )
        self._startup_timer = None
        if startup_color_enabled:
            self._set_all_channels(startup_red, startup_green, startup_blue)
            self._startup_timer = self.create_timer(startup_delay_sec, self._publish_startup_color)

    def _publish_channels(self) -> None:
        red_msg = Float64MultiArray()
        red_msg.data = self._red_values
        self._red_pub.publish(red_msg)

        green_msg = Float64MultiArray()
        green_msg.data = self._green_values
        self._green_pub.publish(green_msg)

        blue_msg = Float64MultiArray()
        blue_msg.data = self._blue_values
        self._blue_pub.publish(blue_msg)

    @staticmethod
    def _clamp_channel(value: int) -> float:
        return float(max(0, min(255, value)))

    def _set_all_channels(self, red: int, green: int, blue: int) -> None:
        self._red_values = [self._clamp_channel(red)] * len(self._actuator_names)
        self._green_values = [self._clamp_channel(green)] * len(self._actuator_names)
        self._blue_values = [self._clamp_channel(blue)] * len(self._actuator_names)

    def _publish_startup_color(self) -> None:
        if not self._startup_color_active:
            return

        self._publish_channels()
        self._startup_publish_remaining -= 1
        self.get_logger().info(
            f"Published startup Dynamixel LED color ({self._startup_publish_remaining} retries left)"
        )
        if self._startup_publish_remaining <= 0 and self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

    def _on_colors(self, msg: DynamixelLedColors) -> None:
        updated = False
        for color in msg.colors:
            if color.actuator_name not in self._actuator_indices:
                self.get_logger().warning(
                    f"Unknown actuator in DynamixelLedColors: {color.actuator_name}"
                )
                continue

            index = self._actuator_indices[color.actuator_name]
            self._red_values[index] = float(color.red)
            self._green_values[index] = float(color.green)
            self._blue_values[index] = float(color.blue)
            updated = True

        if updated:
            self._startup_color_active = False
            if self._startup_timer is not None:
                self._startup_timer.cancel()
                self._startup_timer = None
            self._publish_channels()


def main() -> None:
    rclpy.init()
    node = DynamixelLedColorsBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
