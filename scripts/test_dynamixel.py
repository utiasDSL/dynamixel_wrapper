from dynamixel_wrapper import DynamixelMotor
import time

motor = DynamixelMotor(
    model_name="XC430-W150T",
    id=1,
    device_name="/dev/ttyUSB1",
)

motor.connect()

# %%
motor.connect()

motor.packet_handler.factoryReset(motor.port_handler, motor.id, 0)
# %%
motor.set_torque_enable(False)

motor.reboot()
# %%
pos = motor.get_position()
print(f"Current position: {pos}")

# %%
delta = 10
max_delta = 100
direction = 1
motor.set_torque_enable(True)
motor.set_position(2500)
time.sleep(1)
pos = motor.get_position()
while True:
    if pos >= 2600:
        direction = -1
    elif pos <= 2400:
        direction = 1
    pos = motor.get_position()
    print(f"Current position: {pos}")
    new_pos = pos + direction * max_delta
    motor.set_position(new_pos)
    time.sleep(0.005)


# %%
motor.set_torque_enable(True)

# %%
pos = motor.get_position()
motor.set_position(1400)
while True:
    pos = motor.get_position()
    print(f"Current position: {pos}")
    if abs(pos - 1600) < 5:
        break
    time.sleep(0.01)

# %%
