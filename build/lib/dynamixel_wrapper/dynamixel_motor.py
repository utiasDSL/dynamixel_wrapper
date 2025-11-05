"""Dynamixel motor control wrapper using dynamixel_sdk."""

from dynamixel_sdk import PortHandler, PacketHandler
from dynamixel_wrapper.dynamixel_table import make_dynamixel_table


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
        if dxl_comm_result != 0:
            raise Exception(f"Communication error: {dxl_comm_result}, {dxl_error}")

    def get_position(self):
        """Get the current position of the motor."""
        present_position, dxl_comm_result, dxl_error = (
            self.packet_handler.read4ByteTxRx(
                self.port_handler,
                self.id,
                self.table_class.ADDR_PRESENT_POSITION,
            )
        )
        if dxl_comm_result != 0:
            raise Exception(f"Communication error: {dxl_comm_result}, {dxl_error}")
        return present_position

    def set_torque_enable(self, enable: bool):
        """Enable or disable the torque of the motor."""
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_TORQUE_ENABLE,
            1 if enable else 0,
        )
        if dxl_comm_result != 0:
            raise Exception(f"Communication error: {dxl_comm_result}, {dxl_error}")

    def is_torque_enabled(self):
        """Check if the torque is enabled."""
        torque_enable, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            self.id,
            self.table_class.ADDR_TORQUE_ENABLE,
        )
        if dxl_comm_result != 0:
            raise Exception(f"Communication error: {dxl_comm_result}, {dxl_error}")
        return torque_enable == 1
