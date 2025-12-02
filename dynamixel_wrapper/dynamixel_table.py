"""Dynamixel motor address table definitions and registry."""

from abc import ABC
from typing import ClassVar

motor_model_registry = {}


def register_motor_model(model_name: str):
    def decorator(cls):
        motor_model_registry[model_name] = cls
        return cls

    return decorator


class DynamixelAdressTable(ABC):
    ADDR_DRIVE_MODE: ClassVar[int]
    ADDR_OPERATING_MODE: ClassVar[int]

    ADDR_TORQUE_ENABLE: ClassVar[int]
    ADDR_GOAL_POSITION: ClassVar[int]
    ADDR_PRESENT_POSITION: ClassVar[int]
    ADDR_PRESENT_VELOCITY: ClassVar[int]

    ADDR_POSITION_D_GAIN: ClassVar[int]
    ADDR_POSITION_I_GAIN: ClassVar[int]
    ADDR_POSITION_P_GAIN: ClassVar[int]
    ADDR_VELOCITY_P_GAIN: ClassVar[int]
    ADDR_VELOCITY_I_GAIN: ClassVar[int]

    def __init_subclass__(cls):
        required_attributes = {
            name
            for name in DynamixelAdressTable.__annotations__.keys()
            if name.startswith("ADDR_")
        }

        for address in required_attributes:
            if not hasattr(cls, address):
                raise NotImplementedError(
                    f"Subclass {cls.__name__} must implement address {address}"
                )


@register_motor_model("XC430-W150T")
class DynamixelModelXC430W150T(DynamixelAdressTable):
    ADDR_DRIVE_MODE = 10
    ADDR_OPERATING_MODE = 11

    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    ADDR_PRESENT_VELOCITY = 128

    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_VELOCITY_P_GAIN = 78
    ADDR_VELOCITY_I_GAIN = 76
    ADDR_PRESENT_LOAD = 126
    # ADDR_VELOCITY_LIMIT = 44
    ADDR_PWM_LIMIT = 36
    ADDR_GOAL_PWM = 100
    # ADDR_MIN_VOLTAGE_LIMIT = 32
    # ADDR_MAX_VOLTAGE_LIMIT = 34
    # ADDR_TEMPERATURE_LIMIT = 31
    ADDR_HARDWARE_ERROR_STATUS = 70

def make_dynamixel_table(model_name: str) -> DynamixelAdressTable:
    if model_name not in motor_model_registry:
        raise ValueError(f"Motor model '{model_name}' is not registered.")
    return motor_model_registry[model_name]()
