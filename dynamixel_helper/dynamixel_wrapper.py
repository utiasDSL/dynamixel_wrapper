from dynamixel_sdk import PortHandler, PacketHandler


class DynamixelAdressTable:
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132


class DynamixelMotorWrapper:
    def __init__(
        self,
        id: int,
        device_name: str,
        baudrate: int = 57600,
    ):
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
            DynamixelAdressTable.ADDR_GOAL_POSITION,
            position,
        )
        if dxl_comm_result != 0:
            raise Exception(f"Communication error: {dxl_comm_result}, {dxl_error}")

    def get_position(self):
        """Get the current position of the motor."""
        present_position, dxl_comm_result, dxl_error = (
            self.packet_handler.read4ByteTxRx(
                self.port_handler, self.id, DynamixelAdressTable.ADDR_PRESENT_POSITION
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
            DynamixelAdressTable.ADDR_TORQUE_ENABLE,
            1 if enable else 0,
        )
        if dxl_comm_result != 0:
            raise Exception(f"Communication error: {dxl_comm_result}, {dxl_error}")

    def is_torque_enabled(self):
        """Check if the torque is enabled."""
        torque_enable, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler, self.id, DynamixelAdressTable.ADDR_TORQUE_ENABLE
        )
        if dxl_comm_result != 0:
            raise Exception(f"Communication error: {dxl_comm_result}, {dxl_error}")
        return torque_enable == 1
