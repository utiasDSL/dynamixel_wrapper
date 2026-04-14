#!/usr/bin/env python3
"""Automatic Motor Calibration Script

This script calibrates a Dynamixel motor by moving it to its physical limits (stoppers, grippers limits, etc.)
in both directions. It incrementally moves the motor until it reaches the end
of travel (detected by position error threshold and zero velocity).
"""

import time
import argparse
from dynamixel_wrapper import DynamixelMotor


class MotorCalibrator:
    def __init__(
        self,
        motor: DynamixelMotor,
        position_threshold: int = 50,
        velocity_threshold: int = 5,
        increment: int = 50,
        delay: float = 0.1,
    ):
        """Initialize the motor calibrator.

        Args:
            motor: DynamixelMotor instance to calibrate
            position_threshold: Max distance between goal and current position to consider reached (default: 10)
            velocity_threshold: Max velocity to consider motor stopped (default: 5)
            increment: Position increment per step (default: 50)
            delay: Delay between position updates in seconds (default: 0.1)
        """
        self.motor = motor
        self.position_threshold = position_threshold
        self.velocity_threshold = velocity_threshold
        self.increment = increment
        self.delay = delay

    def is_motor_at_limit(self, goal_position: int) -> bool:
        """Check if motor has reached its physical limit.

        A motor is considered at its limit when:
        1. Distance between current and goal position exceeds threshold
        2. Current velocity is approximately zero

        Args:
            goal_position: The target position we're trying to reach

        Returns:
            True if motor is at limit, False otherwise
        """
        current_position = self.motor.get_position()
        current_velocity = abs(self.motor.get_velocity())

        position_error = abs(current_position - goal_position)

        at_limit = (
            position_error >= self.position_threshold
            and current_velocity <= self.velocity_threshold
        )

        if at_limit:
            print(
                f"  Limit detected: pos_error={position_error}, velocity={current_velocity}"
            )

        return at_limit

    def move_to_limit(self, direction: str, start_position: int) -> int:
        """Move motor incrementally in specified direction until limit is reached.

        Args:
            direction: "down" (decreasing position) or "up" (increasing position)
            start_position: Initial position to start from

        Returns:
            Final position (limit position)
        """
        if direction not in ["down", "up"]:
            raise ValueError("Direction must be 'down' or 'up'")

        increment = -self.increment if direction == "down" else self.increment

        print(f"\n{'=' * 60}")
        print(f"Moving {direction.upper()} to find limit...")
        print(f"{'=' * 60}")
        print(f"Start position: {start_position}")
        print(f"Increment: {increment}")
        print(f"Position threshold: {self.position_threshold}")
        print(f"Velocity threshold: {self.velocity_threshold}")
        print()

        current_goal = start_position
        step = 0

        while True:
            current_goal += increment

            # Set new goal position
            self.motor.set_position(current_goal)

            # Wait for motor to attempt movement
            time.sleep(self.delay)

            # Check current state
            current_position = self.motor.get_position()
            current_velocity = self.motor.get_velocity()
            position_error = abs(current_position - current_goal)

            step += 1
            print(
                f"Step {step}: goal={current_goal}, current={current_position}, "
                f"error={position_error}, velocity={abs(current_velocity)}"
            )

            if self.is_motor_at_limit(current_goal):
                break

        print(f"\n✓ Limit reached at position: {current_position}")
        self.motor.set_position(current_position)
        return current_position

    def calibrate(self) -> dict:
        """Perform full calibration by finding both limits.

        Returns:
            Dictionary with calibration results:
                - 'min_value': minimum position (down limit)
                - 'max_value': maximum position (up limit)
        """
        print("\n" + "=" * 60)
        print("MOTOR CALIBRATION STARTING")
        print("=" * 60)

        # Get initial position
        initial_position = self.motor.get_position()
        print(f"\nInitial position: {initial_position}")

        # Enable torque
        if not self.motor.is_torque_enabled():
            print("Enabling torque...")
            self.motor.set_torque_enable(True)

        lower_limit = self.move_to_limit("down", initial_position)
        upper_limit = self.move_to_limit("up", lower_limit)

        # Calculate results
        motion_range = abs(lower_limit - upper_limit)
        center_position = (lower_limit + upper_limit) // 2

        results = {
            "min_value": lower_limit,
            "max_value": upper_limit,
        }

        # Print summary
        print("\n" + "=" * 60)
        print("CALIBRATION COMPLETE")
        print("=" * 60)
        print(f"Lower limit:   {lower_limit}")
        print(f"Upper limit:  {upper_limit}")
        print(f"Range:        {motion_range}")
        print(f"Center:       {center_position}")
        print("=" * 60 + "\n")

        # Move to center
        print("Moving to center position...")
        self.motor.set_position(center_position)
        time.sleep(1.0)

        return results


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate a Dynamixel motor by finding its physical limits"
    )
    parser.add_argument(
        "--device",
        type=str,
        help="Serial device name (e.g.: /dev/ttyUSB0)",
    )
    parser.add_argument("--motor-id", type=int, default=0, help="Motor ID (default: 0)")
    parser.add_argument(
        "--model",
        type=str,
        default="XC430-W150T",
        help="Motor model name (default: XC430-W150T)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=57600,
        help="Communication baudrate (default: 57600)",
    )
    parser.add_argument(
        "--position-threshold",
        type=int,
        default=50,
        help="Position error threshold for limit detection (default: 50)",
    )
    parser.add_argument(
        "--velocity-threshold",
        type=int,
        default=1,
        help="Velocity threshold for stopped detection (default: 1)",
    )
    parser.add_argument(
        "--increment",
        type=int,
        default=20,
        help="Position increment per step (default: 20)",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.1,
        help="Delay between position updates in seconds (default: 0.1)",
    )

    args = parser.parse_args()

    # Create and connect motor
    print("Connecting to motor...")
    print(f"  Device: {args.device}")
    print(f"  Motor ID: {args.motor_id}")
    print(f"  Model: {args.model}")
    print(f"  Baudrate: {args.baudrate}")

    motor = DynamixelMotor(
        model_name=args.model,
        id=args.motor_id,
        device_name=args.device,
        baudrate=args.baudrate,
    )

    try:
        motor.connect()
        print("✓ Motor connected successfully\n")

        calibrator = MotorCalibrator(
            motor=motor,
            position_threshold=args.position_threshold,
            velocity_threshold=args.velocity_threshold,
            increment=args.increment,
            delay=args.delay,
        )

        results = calibrator.calibrate()

        print("\n" + "!" * 70)
        print("IMPORTANT: you need to set those limits in the")
        print("  scripts/dynamixel_motor_ros2.py")
        print("file AND in your crisp_py / crisp_gym config.")
        print("Values to copy into POSITION_LIMITS:")
        print(f'    "{args.device}": ({results["min_value"]}, {results["max_value"]}),')
        print("!" * 70 + "\n")

    except Exception as e:
        print(f"Error during calibration: {e}")
        raise

    finally:
        print("\nDisconnecting motor...")
        motor.disconnect()
        print("✓ Motor disconnected")


if __name__ == "__main__":
    main()
