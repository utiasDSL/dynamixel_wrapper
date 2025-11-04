from dynamixel_helper.dynamixel_wrapper import DynamixelMotorWrapper

motor = DynamixelMotorWrapper(0, "/dev/gripper_left")
motor.connect()

# %%
motor.set_torque_enable(False)
# %%
pos = motor.get_position()

# %%
motor.set_torque_enable(True)

# %%
pos = motor.get_position()
motor.set_position(pos + 100)
