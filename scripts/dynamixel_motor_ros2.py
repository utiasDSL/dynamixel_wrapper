#!/usr/bin/env python3
"""ROS2 node for controlling Dynamixel motors."""

import time
import argparse
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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
        node_name: str = "dynamixel_motor_node",
        motor_name: str = "dynamixel_motor",
        joint_states_alias_topic: str = "",
        command_alias_topic: str = "",
        set_torque_alias_service: str = "",
        motor_query_rate: float = 20.0,
    ):
        """Initialize the Dynamixel motor ROS2 node."""
        super().__init__(node_name, namespace=namespace)

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
        self.joint_state_publisher_compat = self.create_publisher(
            JointState,
            "joint_states",
            10,
        )
        self.joint_state_publisher_alias = None
        if joint_states_alias_topic:
            self.joint_state_publisher_alias = self.create_publisher(
                JointState,
                joint_states_alias_topic,
                10,
            )

        self.position_cmd_subscriber = self.create_subscription(
            Float64MultiArray,
            f"{motor_name}/command",
            self.position_command_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.position_cmd_subscriber_alias = None
        if command_alias_topic:
            self.position_cmd_subscriber_alias = self.create_subscription(
                Float64MultiArray,
                command_alias_topic,
                self.position_command_callback,
                10,
                callback_group=ReentrantCallbackGroup(),
            )

        timer_period = 1.0 / publish_rate
        # Timer to handle publishing
        self._publish_joints_timer = self.create_timer(
            timer_period,
            self.publish_joint_state,
            callback_group=ReentrantCallbackGroup(),
        )

        # Timers to handle reading and writing positions (one at a time)
        motor_handling_callback_group = MutuallyExclusiveCallbackGroup()

        self._joint_read_timer = self.create_timer(
            1.0 / motor_query_rate,
            self.read_joint_state_callback,
            callback_group=motor_handling_callback_group,
        )
        self._set_target_timer = self.create_timer(
            1.0 / motor_query_rate,
            self.set_target_position_callback,
            callback_group=motor_handling_callback_group,
        )
        self.torque_service = self.create_service(
            SetBool,
            f"{motor_name}/set_torque",
            self.set_torque_callback,
            callback_group=motor_handling_callback_group,
        )
        self.torque_service_alias = None
        if set_torque_alias_service:
            self.torque_service_alias = self.create_service(
                SetBool,
                set_torque_alias_service,
                self.set_torque_callback,
                callback_group=motor_handling_callback_group,
            )

        self._error_occurred = False

        self._latest_position = None
        self._latest_velocity = None
        self.target_position = None

        self.should_torque_be_enabled = False

        self.get_logger().info("Dynamixel motor ROS2 node initialized")

    def publish_joint_state(self):
        """Publish current motor position and velocity as JointState."""
        if self._latest_position is None or self._latest_velocity is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [float(self._latest_position)]
        msg.velocity = [float(self._latest_velocity)]
        msg.effort = [float(0.0)]  # Effort is not provided by the motor

        self.joint_state_publisher.publish(msg)
        # Compatibility publisher for stacks expecting <namespace>/joint_states.
        self.joint_state_publisher_compat.publish(msg)
        if self.joint_state_publisher_alias is not None:
            self.joint_state_publisher_alias.publish(msg)

    def position_command_callback(self, msg: Float64MultiArray):
        """Handle position command messages."""
        if len(msg.data) < 1:
            self.get_logger().warn("Received empty position command")
            return

        self.target_position = int(msg.data[0])

    def set_torque_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle torque enable/disable service requests."""
        try:
            self.should_torque_be_enabled = request.data
            self.motor.set_torque_enable(self.should_torque_be_enabled)
            response.success = True
            response.message = f"Torque {'enabled' if request.data else 'disabled'}"
            self.get_logger().info(response.message)
        except RuntimeError as e:
            response.success = False
            response.message = f"Failed to set torque: {e}"
            self.get_logger().error(response.message)

        return response

    def set_target_position_callback(self):
        if self._error_occurred:
            return
        if self.target_position is None:
            return

        try:
            self.motor.set_position(self.target_position)
        except RuntimeError as e:
            self.get_logger().error(f"Error setting position: {e}")
            self._try_recovery()

    def read_joint_state_callback(self):
        if self._error_occurred:
            return

        try:
            self._latest_position = self.motor.get_position()
            self._latest_velocity = self.motor.get_velocity()
        except RuntimeError as e:
            self.get_logger().error(f"Error reading joint state: {e}")
            self._try_recovery()

    def _try_recovery(self):
        """Attempt to recover from a motor communication error."""
        try:
            self._error_occurred = True

            self.get_logger().warn("Attempting to recover motor connection...")
            self.motor.reboot()
            time.sleep(0.15)
            self.get_logger().info("Motor connection recovered")

            self.motor.set_torque_enable(self.should_torque_be_enabled)

            time.sleep(0.05)
            self._error_occurred = False
        except RuntimeError as e:
            self.get_logger().error(f"Recovery failed: {e}")

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
        "--device",
        dest="device_name",
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
        "--node-name",
        type=str,
        default="dynamixel_motor_node",
        help="ROS2 node name (default: dynamixel_motor_node)",
    )
    parser.add_argument(
        "--motor-name",
        type=str,
        default="dynamixel_motor",
        help="Name of the motor node (default: dynamixel_motor)",
    )
    parser.add_argument(
        "--joint-states-alias-topic",
        type=str,
        default="",
        help="Optional extra JointState topic to publish, e.g. /left/franka_gripper/joint_states",
    )
    parser.add_argument(
        "--command-alias-topic",
        type=str,
        default="",
        help="Optional extra command topic to subscribe, e.g. /left/franka_gripper/command",
    )
    parser.add_argument(
        "--set-torque-alias-service",
        type=str,
        default="",
        help="Optional extra SetBool service name, e.g. /left/franka_gripper/set_torque",
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
        node_name=args.node_name,
        motor_name=args.motor_name,
        joint_states_alias_topic=args.joint_states_alias_topic,
        command_alias_topic=args.command_alias_topic,
        set_torque_alias_service=args.set_torque_alias_service,
    )
    node.get_logger().info("Rebooting motor to ensure clean state...")
    node.motor.reboot()
    time.sleep(0.5)

    node.get_logger().info("Reconnecting to motor after reboot...")
    try:
        node.motor.connect()
    except Exception as e:
        node.get_logger().error(f"Error when connecting: {e}")

    node.get_logger().info("Motor rebooted successfully.")

    try:
        node.motor.set_torque_enable(True)
    except Exception as e:
        node.get_logger().error(f"Error when enabling torque: {e}")
    node.should_torque_be_enabled = True

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            node.get_logger().error(f"Unexpected error: {e}")
            node._try_recovery()

    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
