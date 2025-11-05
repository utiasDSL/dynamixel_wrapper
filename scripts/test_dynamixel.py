from dynamixel_wrapper import DynamixelMotor
import time

motor = DynamixelMotor(
    model_name="XC430-W150T",
    id=0,
    device_name="/dev/gripper_right",
)

motor.connect()

# %%
motor.set_torque_enable(False)

# %%
pos = motor.get_position()
print(f"Current position: {pos}")
motor.set_position(1500)

# %%
motor.set_torque_enable(True)

# %%
pos = motor.get_position()
motor.set_position(1600)
while True:
    pos = motor.get_position()
    print(f"Current position: {pos}")
    if abs(pos - 1600) < 5:
        break
    time.sleep(0.01)
