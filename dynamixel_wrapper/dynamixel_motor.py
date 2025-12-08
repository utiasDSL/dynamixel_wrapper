"""Dynamixel motor control wrapper using dynamixel_sdk."""

from dynamixel_sdk import COMM_SUCCESS, PortHandler, PacketHandler
from dynamixel_wrapper.dynamixel_table import make_dynamixel_table
import time

class DynamixelMotor:
    def __init__(
        self,
        model_name: str,
        id: int,
        device_name: str,
        baudrate: int = 57600,
    ):
        """Initialize the Dynamixel motor with the given parameters.
        Args:
            model_name (str): The model name of the Dynamixel motor.
            id (int): The ID of the motor.
            device_name (str): The device name for the port (e.g., '/dev/ttyUSB0').
            baudrate (int, optional): The baudrate for communication. Defaults to 57600.
        """
        self.model_name = model_name
        self.table_class = make_dynamixel_table(model_name)

        self.id = id
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(2.0)
        self.baudrate = baudrate

    def connect(self):
        if not self.port_handler.openPort():
            raise RuntimeError("Failed to open the port!")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError("Failed to set the baudrate!")

    def set_position(self, position):
        """Set the target position of the motor."""
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_GOAL_POSITION,
            position,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

    def get_position(self):
        """Get the current position of the motor."""
        present_position, dxl_comm_result, dxl_error = (
            self.packet_handler.read4ByteTxRx(
                self.port_handler,
                self.id,
                self.table_class.ADDR_PRESENT_POSITION,
            )
        )
        self._raise_if_error(dxl_comm_result, dxl_error)
        return present_position

    def set_torque_enable(self, enable: bool):
        """Enable or disable the torque of the motor."""
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_TORQUE_ENABLE,
            1 if enable else 0,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

    def is_torque_enabled(self):
        """Check if the torque is enabled."""
        torque_enable, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_TORQUE_ENABLE,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)
        return torque_enable == 1

    def get_position_gains(self):
        """Get the position control gains."""
        p_gain, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_POSITION_P_GAIN,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        i_gain, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_POSITION_I_GAIN,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        d_gain, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_POSITION_D_GAIN,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        return p_gain, i_gain, d_gain

    def get_velocity(self):
        """Get the current velocity of the motor."""
        present_velocity, dxl_comm_result, dxl_error = (
            self.packet_handler.read4ByteTxRx(
                self.port_handler,
                self.id,
                self.table_class.ADDR_PRESENT_VELOCITY,
            )
        )
        self._raise_if_error(dxl_comm_result, dxl_error)
        return present_velocity

    def set_velocity(self, velocity):
        """Set the target velocity of the motor."""
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_GOAL_POSITION,
            velocity,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

    def get_velocity_gains(self):
        """Get the velocity control gains."""
        p_gain, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_VELOCITY_P_GAIN,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        i_gain, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_VELOCITY_I_GAIN,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        return p_gain, i_gain
    
    def get_present_load(self):
        """Get the present_load."""
        present_load, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_PRESENT_LOAD,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        return present_load

    def get_operating_mode(self):
        """Get the Operating Mode."""
        raw_mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_OPERATING_MODE,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)
        return int(raw_mode)

    def switch_operating_mode(self, current_mode: int):
        """Switch the Operating Mode between position control mode and pwm control mode."""
        if current_mode == 3:  # Position Control Mode
            goal_mode = 16  # PWM Control Mode
        elif current_mode == 16:  # PWM Control Mode
            goal_mode = 3  # Position Control Mode
        self.set_torque_enable(False)  # Disable torque before switching mode
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_OPERATING_MODE,
            goal_mode,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)  
        self.set_torque_enable(True)  # Disable torque before switching mode  
        time.sleep(0.05)
        return goal_mode

    def get_goal_pwm(self):
        """Get the goal pwm."""
        goal_pwm, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_GOAL_PWM,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        return goal_pwm
    def get_pwm_limit(self):
        """Get the PWM limit."""
        pwm_limit, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_PWM_LIMIT,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

        return pwm_limit
    
    def set_pwm_limit(self, goal_pwm: int):
        """Set the PWM limit."""
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_PWM_LIMIT,
            goal_pwm,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

    def set_goal_pwm(self, goal_pwm: int):
        """Set the goal PWM."""
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_GOAL_PWM,
            goal_pwm,
        )
        self._raise_if_error(dxl_comm_result, dxl_error)

    def set_position_gains(self, p_gain: int, i_gain: int, d_gain: int):
        """Set the position control gains."""
        for addr, gain in [
            (self.table_class.ADDR_POSITION_P_GAIN, p_gain),
            (self.table_class.ADDR_POSITION_I_GAIN, i_gain),
            (self.table_class.ADDR_POSITION_D_GAIN, d_gain),
        ]:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler,
                self.id,
                addr,
                gain,
            )
            self._raise_if_error(dxl_comm_result, dxl_error)

    def disconnect(self):
        """Close the port connection."""
        self.set_torque_enable(enable=False)
        self.port_handler.closePort()

    def _raise_if_error(self, dxl_comm_result: int, dxl_error: int):
        """Raise an exception if there was a communication or Dynamixel error."""
        errors = []
        if dxl_comm_result != COMM_SUCCESS:
            errors.append(
                f"Communication error: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )

        if dxl_error != 0:
            errors.append(
                f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}"
            )

        if errors:
            raise RuntimeError("; ".join(errors))
