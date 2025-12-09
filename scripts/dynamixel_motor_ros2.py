#!/usr/bin/env python3
"""ROS2 node for controlling Dynamixel motors."""

import time
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from dynamixel_wrapper import DynamixelMotor


class DynamixelMotorROS2Node(Node):
    """ROS2 node for Dynamixel motor control."""

    def __init__(
        self,
        model_name: str,
        motor_id: int,
        device_name: str,
        baudrate: int,
        joint_name: str,
        publish_rate: float,
        namespace: str = "",
        motor_name: str = "dynamixel_motor",
    ):
        """Initialize the Dynamixel motor ROS2 node."""
        super().__init__("dynamixel_motor_node", namespace=namespace)

        self.joint_name = joint_name

        self.motor = DynamixelMotor(
            model_name=model_name,
            id=motor_id,
            device_name=device_name,
            baudrate=baudrate,
        )

        try:
            self.motor.connect()
            self.get_logger().info(
                f"Connected to Dynamixel motor {motor_id} on {device_name}"
            )
        except RuntimeError as e:
            self.get_logger().error(f"Failed to connect to motor: {e}")
            raise

        self.joint_state_publisher = self.create_publisher(
            JointState,
            f"{motor_name}/joint_states",
            10,
        )

        self.position_cmd_subscriber = self.create_subscription(
            Float64MultiArray,
            f"{motor_name}/command",
            self.position_command_callback,
            10,
        )

        self.torque_service = self.create_service(
            SetBool,
            f"{motor_name}/set_torque",
            self.set_torque_callback,
        )

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

        self._error_occurred = False

        self._previous_position = None
        self._previous_velocity = None
        self.target_position = None

        self.should_torque_be_enabled = False

        self.get_logger().info("Dynamixel motor ROS2 node initialized")

    def publish_joint_state(self):
        """Publish current motor position and velocity as JointState."""

        try:
            self._previous_position = (
                self.motor.get_position()
                if not self._error_occurred
                else self._previous_position
            )
            self._previous_velocity = (
                self.motor.get_velocity()
                if not self._error_occurred
                else self._previous_velocity
            )

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self.joint_name]
            msg.position = [float(self._previous_position)]
            msg.velocity = [float(self._previous_velocity)]
            msg.effort = [float(0.0)]  # Effort is not provided by the motor

            self.joint_state_publisher.publish(msg)
        except RuntimeError as e:
            self.get_logger().error(f"Error reading motor state: {e}")
            self._try_recovery()

    def position_command_callback(self, msg: Float64MultiArray):
        """Handle position command messages."""
        if len(msg.data) < 1:
            self.get_logger().warn("Received empty position command")
            return

        self.target_position = int(msg.data[0])

        if self._error_occurred:
            return

        try:
            self.motor.set_position(self.target_position)
        except RuntimeError as e:
            self.get_logger().error(f"Error setting position: {e}")
            self._try_recovery()

    def set_torque_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle torque enable/disable service requests."""
        try:
            self._should_torque_be_enabled = request.data
            self.motor.set_torque_enable(self._should_torque_be_enabled)
            response.success = True
            response.message = f"Torque {'enabled' if request.data else 'disabled'}"
            self.get_logger().info(response.message)
        except RuntimeError as e:
            response.success = False
            response.message = f"Failed to set torque: {e}"
            self.get_logger().error(response.message)

        return response

    def _try_recovery(self):
        """Attempt to recover from a motor communication error."""
        self._error_occurred = True

        self.get_logger().warn("Attempting to recover motor connection...")
        self.motor.reboot()
        time.sleep(0.15)
        self.get_logger().info("Motor connection recovered")

        self.motor.set_torque_enable(self.should_torque_be_enabled)

        time.sleep(0.01)
        self._error_occurred = False

    def shutdown(self):
        """Cleanup on node shutdown."""
        self.get_logger().info("Shutting down Dynamixel motor node")
        try:
            self.motor.disconnect()
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 node for Dynamixel motor control"
    )
    parser.add_argument(
        "--model-name",
        type=str,
        default="XC430-W150T",
        help="Motor model name (default: XC430-W150T)",
    )
    parser.add_argument(
        "--motor-id",
        type=int,
        default=0,
        help="Motor ID (default: 0)",
    )
    parser.add_argument(
        "--device-name",
        type=str,
        default="/dev/gripper_left",
        help="Serial device name (default: /dev/gripper_left)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=57600,
        help="Communication baudrate (default: 57600)",
    )
    parser.add_argument(
        "--joint-name",
        type=str,
        default="dynamixel_joint",
        help="Joint name for JointState messages (default: dynamixel_joint)",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=50.0,
        help="Publishing rate in Hz (default: 50.0)",
    )
    parser.add_argument(
        "--namespace",
        type=str,
        default="",
        help="ROS2 namespace for topics and services (default: '')",
    )
    parser.add_argument(
        "--motor-name",
        type=str,
        default="dynamixel_motor",
        help="Name of the motor node (default: dynamixel_motor)",
    )

    args = parser.parse_args()

    rclpy.init()

    node = DynamixelMotorROS2Node(
        model_name=args.model_name,
        motor_id=args.motor_id,
        device_name=args.device_name,
        baudrate=args.baudrate,
        joint_name=args.joint_name,
        publish_rate=args.publish_rate,
        namespace=args.namespace,
        motor_name=args.motor_name,
    )
    node.motor.set_torque_enable(True)
    node.should_torque_be_enabled = True

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
