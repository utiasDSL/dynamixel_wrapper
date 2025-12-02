#!/usr/bin/env python3
"""ROS2 node for controlling Dynamixel motors."""

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

        self.get_logger().info("Dynamixel motor ROS2 node initialized")
        self.last_switch_time = None
        self.pwm_switch_delay = 1.0  # seconds 

    def publish_joint_state(self):
        """Publish current motor position and velocity as JointState."""
        try:
            position = self.motor.get_position()
            velocity = self.motor.get_velocity()

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self.joint_name]
            msg.position = [float(position)]
            msg.velocity = [float(velocity)]
            msg.effort = [float(0.0)]  # Effort is not provided by the motor

            self.joint_state_publisher.publish(msg)
        except RuntimeError as e:
            self.get_logger().error(f"Error reading motor state: {e}")

    def position_command_callback(self, msg: Float64MultiArray):
        """Handle position command messages."""
        if len(msg.data) < 1:
            self.get_logger().warn("Received empty position command")
            return

        # try:
        #     target_position = int(msg.data[0])
        #     self.motor.set_position(target_position)
        # except RuntimeError as e:
        #     self.get_logger().error(f"Error setting position: {e}")



        load = self.motor.get_present_load()  # raw 16-bit value
        self.get_logger().info(f"Present Load: {load}")
        # self.get_logger().info(f"Target Position: {target_position}")




        # change to pwm control if grasping to avoid overload

        try:  
            target_position = int(msg.data[0])
            operating_mode = self.motor.get_operating_mode()


            
            if operating_mode == 16:  # PWM mode
                if int(msg.data[0]) > self.motor.get_position()+20:
                    self.get_logger().info(f"current mode 16, switching to 3") 
                    operating_mode = self.motor.switch_operating_mode(operating_mode)
                    self.get_logger().info(f"Operating Mode switched successfully: {operating_mode}") 
                    self.motor.set_position(target_position)
                else:
                    self.get_logger().info(f"pwm mode") 
                    self.motor.set_goal_pwm(-350) # set to 100%
                    self.get_logger().info(f"{self.motor.get_goal_pwm()}")
            elif operating_mode == 3:  # Position control mode
                now = self.get_clock().now().nanoseconds * 1e-9
                if target_position < self.motor.get_position()-100:
                    if self.last_switch_time is None:
                        self.last_switch_time = now
                    self.motor.set_position(target_position)
                    if now - self.last_switch_time >= self.pwm_switch_delay:
                        # Delay passed → switch to PWM
                        operating_mode = self.motor.switch_operating_mode(operating_mode)
                        self.get_logger().info(f"Switch to PWM mode after {self.pwm_switch_delay}s")
                        self.last_switch_time = None
                        self.motor.set_goal_pwm(-350)  # Apply full PWM
                else:
                    self.last_switch_time = None
                    self.motor.set_position(target_position)
        except RuntimeError as e:
            self.get_logger().error(f"Error setting position: {e}")

    def set_torque_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle torque enable/disable service requests."""
        try:
            self.motor.set_torque_enable(request.data)
            response.success = True
            response.message = f"Torque {'enabled' if request.data else 'disabled'}"
            self.get_logger().info(response.message)
        except RuntimeError as e:
            response.success = False
            response.message = f"Failed to set torque: {e}"
            self.get_logger().error(response.message)

        return response

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
    node.motor.set_torque_enable(False)
    # node.motor.set_position_gains(1600,0,0)
    node.motor.set_pwm_limit(880)
    print(node.motor.get_pwm_limit())
    # print(node.motor.get_position_gains())
    node.motor.set_torque_enable(True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # print hardware error status
        try:
            hardware_error, dxl_comm_result, dxl_error = node.motor.packet_handler.read1ByteTxRx(
                node.motor.port_handler,
                node.motor.id,
                node.motor.table_class.ADDR_HARDWARE_ERROR_STATUS,
            )
            print(f"Abaaba: {hardware_error:08b}")
        except Exception as e:
            print(f"Babababa: {e}")

        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
