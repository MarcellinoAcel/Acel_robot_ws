import odrive
from odrive.enums import *
import time
# Connect to ODrive
odrv0 = odrive.find_any()

odrv0.axis0.config.enable_sensorless_mode = True

odrv0.axis1.config.enable_sensorless_mode = True
# Disable watchdog timer to prevent timeout issues
odrv0.axis0.config.watchdog_timeout = 0
odrv0.axis0.error = 0
odrv0.axis0.motor.error = 0

odrv0.axis1.config.watchdog_timeout = 0
odrv0.axis1.error = 0
odrv0.axis1.motor.error = 0

odrv0.config.brake_resistance = 2
odrv0.config.dc_max_positive_current = 15
odrv0.config.dc_max_negative_current = -15 #-2
odrv0.config.max_regen_current = 0

odrv0.axis0.controller.config.pos_gain = 20
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.01
odrv0.axis0.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_limit = 1000  # Lower velocity limit
odrv0.axis0.motor.config.current_lim = 15  # Lower current limit to avoid overcurrent violation
odrv0.axis0.motor.config.pole_pairs = 7  # Set correct pole pairs for your motor
# odrv0.axis0.controller.config.vel_ramp_rate = 10  # Velocity ramp rate in counts/s^2
# odrv0.axis0.motor.config.direction = 1

odrv0.axis1.controller.config.pos_gain = 20
odrv0.axis1.controller.config.vel_gain = 0.02
odrv0.axis1.controller.config.vel_integrator_gain = 0.01
odrv0.axis1.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_limit = 1000  # Lower velocity limit
odrv0.axis1.motor.config.current_lim = 15  # Lower current limit to avoid overcurrent violation
odrv0.axis1.motor.config.pole_pairs = 7  # Set correct pole pairs for your motor
# odrv0.axis0.controller.config.vel_ramp_rate = 10  # Velocity ramp rate in counts/s^2
# odrv0.axis0.motor.config.direction = 1

odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422/(7*190)
odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.motor.config.pre_calibrated = True

odrv0.axis1.sensorless_estimator.config.pm_flux_linkage = 5.51328895422/(7*190)
odrv0.axis1.requested_state = odrive.enums.AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis1.motor.config.pre_calibrated = True

# odrv0.axis0.controller.input_vel = 50
# odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
# while True:
#     odrv0.axis0.controller.input_vel = 70
#     time.sleep(5)
#     odrv0.axis0.controller.input_vel = 20
#     time.sleep(5)
# odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_SENSORLESS_CONTROL
# while True :
#     odrv0.axis0.requested_state = odrive.enums.AXIS_STATE_SENSORLESS_CONTROL
#     odrv0.axis0.controller.input_vel = 5

#     # Check for any errors
#     print("Axis 0 errors:", hex(odrv0.axis0.error))
#     print("Motor errors:", hex(odrv0.axis0.motor.error))
#     # time.sleep(5)