"""Shared robot-side constants.

These are starter values so the software can be structured before the
physical robot is finished. Expect to tune most of them once hardware exists.
"""

from math import pi

FIELD_UNITS = "inches"
FIELD_WIDTH_IN = 96.0
FIELD_HEIGHT_IN = 60.0

# Platform details known so far.
DRIVE_BATTERY_VOLTAGE = 12.0
GEAR_RATIO = 1.0
RASPBERRY_PI_MODEL = "Raspberry Pi 3 Model B v1.2"
ARDUINO_MODEL = "Arduino Uno"
MOTOR_MODEL = "Tsiny geared motor"
MOTOR_NOMINAL_VOLTAGE = 12.0
MOTOR_FREE_SPEED_RPM = 300.0
DEFAULT_SERIAL_PORT_LINUX = "/dev/ttyACM0"
DEFAULT_SERIAL_PORT_WINDOWS = "COM3"

# Current chassis assumptions based on the planned build.
WHEEL_DIAMETER_IN = 2.0
WHEEL_WIDTH_IN = 1.0
ROBOT_LENGTH_IN = 4.5
ROBOT_WIDTH_IN = 3.5
WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * pi
MOTOR_FREE_SPEED_RPS = MOTOR_FREE_SPEED_RPM / 60.0
THEORETICAL_MAX_WHEEL_SPEED_IN_PER_S = WHEEL_CIRCUMFERENCE_IN * MOTOR_FREE_SPEED_RPS / GEAR_RATIO

# Distance between wheel centers along each robot axis.
FRONT_BACK_WHEEL_SPACING_IN = ROBOT_LENGTH_IN
SIDE_TO_SIDE_WHEEL_SPACING_IN = ROBOT_WIDTH_IN

# Kept for compatibility with earlier differential-drive helpers.
TRACK_WIDTH_IN = SIDE_TO_SIDE_WHEEL_SPACING_IN

# Control loop timing.
DEFAULT_CONTROL_DT_S = 0.02

# Simulated chassis response for pre-hardware tuning.
SIM_MAX_FORWARD_SPEED_IN_PER_S = 8.0
SIM_MAX_STRAFE_SPEED_IN_PER_S = 6.0
SIM_MAX_STEPS_PER_RUN = 120

# Translation PID starter gains.
DRIVE_KP = 0.8
DRIVE_KI = 0.0
DRIVE_KD = 0.08

# Heading PID starter gains.
TURN_KP = 2.2
TURN_KI = 0.0
TURN_KD = 0.12

# Safety limits for controller outputs and integral accumulation.
MAX_DRIVE_OUTPUT = 1.0
MAX_TURN_OUTPUT = 1.0
MAX_INTEGRAL = 10.0
