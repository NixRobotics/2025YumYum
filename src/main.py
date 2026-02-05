# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Nick                                                         #
# 	Created:      09/11/2025, 7:00:00 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #
#   YumYum                                                                     #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from math import pi, cos, sin, radians, degrees
#from enum import Enum
from v5pythonlibrary import *

ALLIANCE_COLOR = AllianceColor.RED
AUTON_SEQUENCE = AutonSequence.SKILLS

# declare devices
brain = Brain()
controller_1 = Controller(PRIMARY)

initialization_complete = False # wait for pre_auton to complete
tests_running = False

left_front_motor = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
left_mid_motor = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
left_back_motor = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)

right_front_motor = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False)
right_mid_motor = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
right_back_motor = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
DRIVETRAIN_WHEEL_SIZE = 257.83 # Wheel circumherence in MM
DRIVETRAIN_TRACK_WIDTH = 320.0 # Not used
DRIVETRAIN_WHEEL_BASE = 320.0 # Not used
DRIVETRAIN_GEAR_RATIO = 36.0/48.0

ramp_motor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
shooter_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)

GYRO_SCALE = (360.0 + 3.475) / 360.0# this is roughly how much robot over-turns per revolution
inertial_sensor = InertialWrapper(Ports.PORT16, GYRO_SCALE)

flippy_solenoid = DigitalOut(brain.three_wire_port.h)
trapdoor_solenoid = DigitalOut(brain.three_wire_port.g)

color1 = Optical(Ports.PORT6)
color2 = Optical(Ports.PORT7)

# vex helper constructs
left_motor_group = MotorGroup(left_front_motor, left_mid_motor, left_back_motor)
right_motor_group = MotorGroup(right_front_motor, right_mid_motor, right_back_motor)
drivetrain = SmartDriveWrapper(left_motor_group, right_motor_group, inertial_sensor,
                               DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_TRACK_WIDTH, DRIVETRAIN_WHEEL_BASE,
                               DistanceUnits.MM,
                               DRIVETRAIN_GEAR_RATIO)

all_motors = [left_front_motor, left_mid_motor, left_back_motor,
              right_front_motor, right_mid_motor, right_back_motor,
              ramp_motor, shooter_motor]

all_motor_names = ["left_front_motor", "left_mid_motor", "left_back_motor",
                   "right_front_motor", "right_mid_motor", "right_back_motor",
                   "ramp_motor", "shooter_motor"]

# ------------------------------------------------------------ #
# Motor Monitoring
#------------------------------------------------------------ #

# see https://github.com/NixRobotics/V5PythonFramework/blob/main/src/motormonitor.py
motor_monitor = None  # type: MotorMonitor | None

# ------------------------------------------------------------ #
# ROBOT POSITION TRACKING

USING_TRACKING_WHEELS = False
USING_RESAMPLING = False

if USING_TRACKING_WHEELS:
    rotation_fwd = Rotation(Ports.PORT6, False)
    rotation_strafe = Rotation(Ports.PORT7, False)
    all_sensors = [inertial_sensor, rotation_fwd, rotation_strafe]

    ODOMETRY_FWD_SIZE = 219.70
    ODOMETRY_FWD_OFFSET = 19.0
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 0.0
    ODOMETRY_STRAFE_OFFSET = 0.0
    ODOMETRY_STRAFE_GEAR_RATIO = 1.0

else:
    all_sensors = [inertial_sensor]

    ODOMETRY_FWD_SIZE = DRIVETRAIN_WHEEL_SIZE
    ODOMETRY_FWD_OFFSET = 0.0
    ODOMETRY_FWD_GEAR_RATIO = DRIVETRAIN_GEAR_RATIO
    ODOMETRY_STRAFE_SIZE = 0.0
    ODOMETRY_STRAFE_OFFSET = 0.0
    ODOMETRY_STRAFE_GEAR_RATIO = 0.0

tracker = None  # type: Tracking | None

def initialize_tracker():
    global tracker
    starting_location = Tracking.Orientation(0.0, 0.0, 0.0)
    tracker_configuration = Tracking.Configuration(
            fwd_is_odom=USING_TRACKING_WHEELS,
            fwd_wheel_size=ODOMETRY_FWD_SIZE,
            fwd_gear_ratio=ODOMETRY_FWD_GEAR_RATIO,
            fwd_offset=ODOMETRY_FWD_OFFSET,
            side_wheel_size=ODOMETRY_STRAFE_SIZE,
            side_gear_ratio=ODOMETRY_STRAFE_GEAR_RATIO,
            side_offset=ODOMETRY_STRAFE_OFFSET
        )
    if USING_TRACKING_WHEELS:
        tracker_devices = [rotation_fwd, rotation_strafe, inertial_sensor]
    else:
        tracker_devices = [left_motor_group, right_motor_group, inertial_sensor]

    tracker = Tracking(tracker_devices, starting_location, tracker_configuration)
    tracker.enable_resampling(USING_RESAMPLING)
    # give tracker some time to get going
    wait(0.1, SECONDS)

def print_tracker(tracker: Tracking, x = 0.0, y = 0.0):
    orientation = tracker.get_orientation()
    origin_distance, origin_heading = tracker.trajectory_to_point(x, y)
    back_x, back_y = tracker.point_on_robot(-160.0, -6.75 * 25.4)
    print("X: {:.1f} mm, Y: {:.1f} mm, Heading: {:.2f} deg".format(orientation.x, orientation.y, orientation.heading))
    print(" - To Point: Distance: {:.1f} mm, Heading: {:.2f} deg".format(origin_distance, origin_heading))
    print(" - Back X: {:.1f} mm, Back Y: {:.1f} mm".format(back_x, back_y))

# ------------------------------------------------------------ #
### ROBOT CONTROL FUNCTIONS ###

## INTAKE/RAMP AND SHOOTER CONTROL ##
# We treat positive values as the"intake" or "up" direction for both intake and conveyor
# Negative values are the "eject" or "down" direction
# Note the small delay when reversing direction. Given the mass of the intake wheels it will act as a flywheel
#  and can potentially damage the motors if we reverse direction too quickly
ramp_speed = 0
shooter_speed = 0

RAMP_MAX_INTAKE = 100
RAMP_MAX_EJECT = -100

SHOOTER_MAX_UP = 100
SHOOTER_MAX_DOWN = -100

# individual control functions
def run_intake(bIntake, speed = 1.0, use_volt = False):
    global ramp_speed
    if ((bIntake and ramp_speed == RAMP_MAX_EJECT) or (not bIntake and ramp_speed == RAMP_MAX_INTAKE)):
        print("reversing intake")
        stop_intake()
        wait(0.1, SECONDS) # wait for intake to slow before reversing direction
    if bIntake: ramp_speed = RAMP_MAX_INTAKE
    else: ramp_speed = RAMP_MAX_EJECT
    if (use_volt):
        volts = ramp_speed * speed * 12.0 / 100.0
        ramp_motor.spin(REVERSE, volts, VoltageUnits.VOLT)
    else:
        ramp_motor.spin(REVERSE, ramp_speed * speed, PercentUnits.PERCENT)

def stop_intake():
    global ramp_speed
    ramp_speed = 0
    ramp_motor.stop(COAST)

def run_shooter(bUp, speed = 1.0):
    global shooter_speed
    if (shooter_speed != 0):
        stop_shooter()
        wait(0.1, SECONDS) # wait for conveyor to slow before reversing direction
    if bUp: shooter_speed = SHOOTER_MAX_UP
    else: shooter_speed = SHOOTER_MAX_DOWN
    shooter_motor.spin(FORWARD, shooter_speed * speed, PERCENT)

def stop_shooter():
    global shooter_speed
    shooter_speed = 0
    shooter_motor.stop(COAST)

# trapdoor control functions
def open_trapdoor():
    trapdoor_solenoid.set(1)

def close_trapdoor():
    trapdoor_solenoid.set(0)

def trapdoor_is_open():
    return trapdoor_solenoid.value() == 1

# flippy control functions
def raise_flippy():
    flippy_solenoid.set(0)

def lower_flippy():
    flippy_solenoid.set(1)

def flippy_is_lowered():
    return flippy_solenoid.value() == 1

# ------------------------------------------------------------ #
# Expermimental color sort function - only scores red right now (rejects blue)

color_detect_count = 0

def enable_color_sort(enable: bool):
    if not enable:
        color1.set_light_power(0, PERCENT)
        color2.set_light_power(0, PERCENT)
    else:
        color1.set_light_power(100, PERCENT)
        color2.set_light_power(100, PERCENT)

COLOR_BLUE_MIN = 200
COLOR_BLUE_MAX = 230
COLOR_RED_MIN = 0
COLOR_RED_MAX = 20

class DetectColor():
    BLUE = 0
    RED = 1

def detect_color(sensor: Optical, color: int):
    '''
    ### Docstring for detect_color
    
    :param sensor: Optical sensor object
    :type sensor: Optical
    :param color: color to detect (DETECT_BLUE or DETECT_RED)
    :type color: int
    '''
    if color == DetectColor.BLUE:
        color_min = COLOR_BLUE_MIN
        color_max = COLOR_BLUE_MAX
    else:
        color_min = COLOR_RED_MIN
        color_max = COLOR_RED_MAX
    if not sensor.is_near_object():
        return False
    hue = sensor.hue()
    if hue > color_min and hue < color_max:
        return True
    return False

def detect_color_both_sensors(color: int):
    if (color == DetectColor.RED):
        return detect_color(color1, color) and detect_color(color2, color)
    else:
        return detect_color(color1, color) or detect_color(color2, color)

color_sort_valid = False
color_sort_valid_count = 0
color_sort_shooter_speed = 0

def color_sort():
    global color_sort_valid, color_sort_valid_count, color_sort_shooter_speed
    reject_color = DetectColor.RED if ALLIANCE_COLOR == AllianceColor.BLUE else DetectColor.BLUE

    if (reject_color== DetectColor.RED) and (detect_color(color1, reject_color) and detect_color(color2, reject_color))\
        or\
        (reject_color == DetectColor.BLUE) and (detect_color(color1, reject_color) or detect_color(color2, reject_color)):

        if not color_sort_valid:
            open_trapdoor()
            color_sort_shooter_speed = shooter_speed
            shooter_motor.stop(COAST)
            print("Reject color detected", reject_color)
        color_sort_valid = True
        color_sort_valid_count = 0

    elif color_sort_valid:
        if color_sort_valid_count < 15:
            color_sort_valid_count += 1
        else:
            close_trapdoor()
            shooter_motor.spin(FORWARD, color_sort_shooter_speed, PERCENT)
            color_sort_valid = False
            print("reject color lost")

def stop_on_color():
    global color_sort_valid, color_sort_valid_count, color_sort_shooter_speed
    reject_color = DetectColor.RED if ALLIANCE_COLOR == AllianceColor.BLUE else DetectColor.BLUE

    detected = False
    if reject_color == DetectColor.RED:
        if detect_color(color1, reject_color) and detect_color(color2, reject_color):
            detected = True

    else:
        if detect_color(color1, reject_color) or detect_color(color2, reject_color):
            detected = True

    if detected:
        # wait(0.15, SECONDS)
        stop_intake()
        stop_shooter()
        return True

    return False

# ------------------------------------------------------------ #
### AUTONOMOUS ###

# This gets set when either autonomous or driver control starts.
# We use this to drop out of the UI
ROBOT_IS_ENABLED = False


def pre_autonomous():
    global initialization_complete
    global motor_monitor
    global ALLIANCE_COLOR, AUTON_SEQUENCE
    # actions to do when the program starts
    # wait a bit before doing anything to let devices initialize
    print("pre-auton code")

    wait(0.1, SECONDS)
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("pre auton code")
    brain.screen.new_line()

    # calibrate the inertial sensor
    if (inertial_sensor.installed):
        inertial_sensor.calibrate()
        while inertial_sensor.is_calibrating():
            wait(10, TimeUnits.MSEC)

    # start background threads
    initialize_tracker()

    wait(0.1, SECONDS)

    print("a piston: ", flippy_solenoid.value())
    print("b piston: ", trapdoor_solenoid.value())

    initialization_complete = True

    ui = PreAutonUI(brain, ALLIANCE_COLOR, AUTON_SEQUENCE)
    ui.start()
    while (not ROBOT_IS_ENABLED):
        ALLIANCE_COLOR, AUTON_SEQUENCE = ui.get_current_selection()
        wait(10, MSEC)
    ui.stop()

    # start motor monitor after pre-auton UI is done
    motor_monitor = MotorMonitor(brain, all_motors, all_motor_names)
    motor_monitor.start()

def drivetrain_max_speeds(motor_speed_rpm, wheel_size_mm, gear_ratio):
    '''
    ### Docstring for drivetrain_max_speeds
    
    :param motor_speed: in RPM (e.g. 600)
    :param wheel_size: circumference in MM (e.g. 260 for 3.25" wheels)
    :param gear_ratio: input gear / output gear (e.g. 24/60)

    :returns linear_speed, turn_speed: Tuple(linear_speed in mm/s, turn_speed in rev/s)
    '''
    linear_speed = motor_speed_rpm * wheel_size_mm * gear_ratio / 60.0 # will be in MM/S 
    turn_speed = 2.0 # hack this for now

    return linear_speed, turn_speed

def setup_drivetrain():
    drivetrain.set_drive_velocity(5, PERCENT)
    drivetrain.set_drive_acceleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_drive_threshold(5) # MM

    drivetrain.set_turn_velocity(40, PERCENT)
    drivetrain.set_turn_constants(Kp=3.0, Ki=0.06, Kd=15.0)
    drivetrain.set_turn_threshold(1) # DEGREES

    drivetrain.set_heading_lock_constants(Kp=4.0)

    drivetrain.set_stopping(BrakeType.BRAKE)

def mid_goal_score():
    run_shooter(True)
    open_trapdoor()
    wait(1.75, SECONDS)
    stop_shooter()

def auto2():
    print("auto2")
    drivetrain.set_drive_velocity(50, PERCENT)
    drivetrain.set_turn_velocity(40, PERCENT)
    drivetrain.set_turn_constants(Kp=3.0, Ki=0.06, Kd=15.0)
    drivetrain.set_turn_threshold(0.25)
    drivetrain.set_stopping(BrakeType.BRAKE)

    revolutions = 1.0
    timeout = revolutions * 3.0
    drivetrain.set_timeout(timeout, SECONDS)
    drivetrain.turn_for(RIGHT, revolutions * 360.0, DEGREES, units_v=PercentUnits.PERCENT, wait=True)
    drivetrain.stop(COAST)

def auto3(tracker: Tracking):
    print("auto3")
    drivetrain.set_drive_velocity(50, PERCENT)
    drivetrain.set_drive_acceleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_heading_lock_constants(5.0)

    drivetrain.set_stopping(BrakeType.BRAKE)

    drivetrain.drive_straight_for(FORWARD, 1200.0, MM, units_v=VelocityUnits.PERCENT, heading=0.0, units_h=DEGREES, wait=True)
    wait(0.1, SECONDS)
    print_tracker(tracker)
    wait(2, SECONDS)
    drivetrain.drive_straight_for(REVERSE, 1200.0, MM, units_v=VelocityUnits.PERCENT, heading=0.0, units_h=DEGREES, wait=True)
    print_tracker(tracker)

def auto4_drive_to_points(tracker: Tracking):
    print("auton4_drive_to_points_long")

    drive_speed = 50 # PERCENT
    turn_speed = 40 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(600, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)

    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_drive_acceleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_drive_threshold(5) # MM

    drivetrain.set_turn_velocity(turn_speed, PERCENT)
    drivetrain.set_turn_constants(Kp=3.0, Ki=0.06, Kd=15.0)
    drivetrain.set_turn_threshold(0.5) # DEGREES

    drivetrain.set_heading_lock_constants(Kp=5.0)

    drivetrain.set_stopping(BrakeType.BRAKE)

    print_tracker(tracker)
    
    x_near = 0.0
    x_far = 2.0 * 600.0

    y_left = 0.0
    y_right = 1.0 * 600.0

    points = [
        [x_near, y_left], # start point
        [x_far, y_left],
        [x_far, y_right],
        [x_near, y_right],
        [x_near, y_left]
    ]
    '''
    # field tiles are 600mm across
    x_near = 1.5 * 600.0
    x_far = 4.5 * 600.0

    y_far_left = 0.5 * 600.0
    y_mid_left = 2.0
    y_mid_right = 4.0 * 600.0
    y_far_right = 5.5 * 600.0

    points = [
        [x_near, y_mid_left], # start point
        [x_far, y_mid_left],
        [x_far, y_mid_right],
        [x_near, y_mid_right],
        [x_near, y_far_left],
        [x_far, y_far_left],
        [x_far, y_far_right],
        [x_near, y_far_right],
        [x_near, y_mid_left]
    ]
    '''

    start_point = points.pop(0) # remove first point as that is the starting point
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    for i in range(4):
        for point in points:
            print("")
            print("----- Start Drive", i, point)
            x = point[0]
            y = point[1]
            print_tracker(tracker, x, y)
            print(inertial_sensor.rotation())
            distance, heading = tracker.trajectory_to_point(x, y)

            drive_timeout = 1.0 + distance / linear_speed_mm_sec # convert to MM/s and pad with 1 sec
            turn_timeout = 1.0 # HACK

            # Point in roughly the right direction
            drivetrain.set_timeout(turn_timeout, SECONDS)
            drivetrain.set_turn_threshold(1.0) # DEGREES - Can relax constrains before longer drives
            drivetrain.turn_to_heading(heading)
            print_tracker(tracker, x, y)

            # Drive straight
            distance, heading = tracker.trajectory_to_point(x, y)
            drivetrain.set_timeout(drive_timeout, SECONDS)
            drivetrain.drive_straight_for(FORWARD, distance, MM, heading=heading)
            wait(0.1, SECONDS)
            print_tracker(tracker, x, y)

    # Final Turn
    print("Start Turn")

    drivetrain.set_timeout(1.0 + turn_timeout, SECONDS)
    drivetrain.set_turn_threshold(0.25) # DEGREES - More accuracy here
    drivetrain.turn_to_heading(0.0)
    
    wait(0.1, SECONDS)
    print_tracker(tracker, start_point[0], start_point[1])

move_done = False
def drive_to_point(tracker: Tracking, x: float, y: float, rev: bool, linear_speed: float, turn_speed: float):
        global move_done
        print_tracker(tracker, x, y)
        print(inertial_sensor.rotation())
        distance, heading = tracker.trajectory_to_point(x, y, reverse=rev)
        # print(distance, heading)

        drive_timeout = 1.0 + abs(distance / linear_speed) # convert to MM/s and pad with 1 sec
        turn_timeout = 1.0 # HACK

        # Point in roughly the right direction
        drivetrain.set_timeout(turn_timeout, SECONDS)
        drivetrain.set_turn_threshold(1.0) # DEGREES - Can relax constrains before longer drives
        drivetrain.turn_to_heading(heading)
        print_tracker(tracker, x, y)

        # Drive straight
        distance, heading = tracker.trajectory_to_point(x, y, reverse=rev)
        drivetrain.set_timeout(drive_timeout, SECONDS)
        drivetrain.drive_straight_for(FORWARD, distance, MM, heading=heading)
        wait(0.1, SECONDS)
        print_tracker(tracker, x, y)
        move_done = True

def auto4_match(tracker: Tracking):
    global move_done
    print("auto4_match")

    drive_speed = 50 # PERCENT
    turn_speed = 50 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(600, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)

    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_drive_acceleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_drive_threshold(5) # MM

    drivetrain.set_turn_velocity(turn_speed, PERCENT)
    drivetrain.set_turn_constants(Kp=3.0, Ki=0.06, Kd=15.0)
    drivetrain.set_turn_threshold(0.5) # DEGREES

    drivetrain.set_heading_lock_constants(Kp=4.0)

    drivetrain.set_stopping(BrakeType.BRAKE)

    print_tracker(tracker)
    
    x_start = 600.0 - 16.75 * 25.4
    y_start = 1200.0 + 8.0 * 25.4

    points = [
        [x_start, y_start, False], # start point
        [900.0, y_start, False],

        [1453.0, 1448.0, True], # align to center goal 
        [1619.0, 1580.0, True], # center goal - 1600,1590
        
        [600.0, 614.0, False], # Align with hopper
        [413.0, 614.0, False], # hopper 600 - 6.75 * 25.4, dot on
        [1179.0-95.0, 600.0, True]
        
    ]

    start_point = points.pop(0) # remove first point as that is the starting point
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    i = 0

    run_intake(True)

    for point in points:
        print("")
        print("----- Start Drive", point)
        x = point[0]
        y = point[1]
        rev = point[2]

        move_done = False
        move_thread = Thread(drive_to_point, (tracker, x, y, rev, linear_speed_mm_sec, turn_speed_rev_sec))
        while not move_done:
            wait(10, MSEC)

        print("done")

        '''
        print_tracker(tracker, x, y)
        print(inertial_sensor.rotation())
        distance, heading = tracker.trajectory_to_point(x, y, reverse=rev)
        # print(distance, heading)

        drive_timeout = 1.0 + abs(distance / linear_speed_mm_sec) # convert to MM/s and pad with 1 sec
        turn_timeout = 1.0 # HACK

        # Point in roughly the right direction
        drivetrain.set_timeout(turn_timeout, SECONDS)
        drivetrain.set_turn_threshold(1.0) # DEGREES - Can relax constrains before longer drives
        drivetrain.turn_to_heading(heading)
        print_tracker(tracker, x, y)

        # Drive straight
        distance, heading = tracker.trajectory_to_point(x, y, reverse=rev)
        drivetrain.set_timeout(drive_timeout, SECONDS)
        drivetrain.drive_straight_for(FORWARD, distance, MM, heading=heading)
        wait(0.1, SECONDS)
        print_tracker(tracker, x, y)
        '''

        if i == 2:
            open_trapdoor()
            wait(1.0, SECONDS)
            close_trapdoor()
            lower_flippy()
            # stop_intake()

        i += 1

    # Final Turn
    '''
    print("Start Turn")

    drivetrain.set_timeout(1.0 + turn_timeout, SECONDS)
    drivetrain.set_turn_threshold(0.25) # DEGREES - More accuracy here
    drivetrain.turn_to_heading(0.0)
    '''

    wait(0.1, SECONDS)
    print_tracker(tracker, start_point[0], start_point[1])

def auton_match_left(tracker: Tracking):
    print("auton_match_left")
    brain.screen.print("auton_match_left")
    brain.screen.new_line()

    reject_color = DetectColor.RED if ALLIANCE_COLOR == AllianceColor.BLUE else DetectColor.BLUE

    setup_drivetrain()
    drive_speed = 50 # PERCENT
    turn_speed = 40 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(600, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)
    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_turn_velocity(turn_speed, PERCENT)

    drive_timer = Timer()
    seq_start_time = drive_timer.time()

    drivetrain.set_stopping(BrakeType.BRAKE)

    print_tracker(tracker)

    x_start = 600.0 - (16.25 - 2.5) * 25.4
    y_start = 1200.0 + (5.25) * 25.4

    points = [
        [x_start, y_start, False], # start point
        [900.0, y_start, False], # 0

        [1250.0, 1250.0, False], # 1: align to center goal 
        [1575.0, 1575.0, True], # 2: center goal - 1600,1590
        
        [600.0, 600.0, False], # 3: Align with hopper
        [385.0, 600.0, False], # 4: hopper 600 - 6.75 * 25.4, dot on
        [1200.0-95.0+20.0, 600.0, True] # 5: score long goal
        
    ]
    start_point = points.pop(0)
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    run_intake(True)

    i = 0 
    for point in points:
        x = point[0]
        y = point[1]
        rev = point[2]

        # pre actions
        turn_timeout = 1.0 # HACK

        if i == 0:
            drivetrain.set_drive_threshold(10.0) # MM

        if i == 1:
            drivetrain.set_drive_threshold(5.0) # MM
            turn_timeout = 0.25

        # Turn phase
        # no turn coming out of parking
        if i > 0:
            distance, heading = tracker.trajectory_to_point(x=x, y=y, reverse=rev)
            drivetrain.set_timeout(turn_timeout, SECONDS)
            drivetrain.turn_to_heading(heading)
            #print_tracker(tracker, x, y)

        # Drive phase
        distance, heading = tracker.trajectory_to_point(x=x, y=y, reverse=rev)
        drive_timeout = 1.0 + abs(distance / linear_speed_mm_sec) # convert to MM/s and pad with 1 sec
        #print("drive_timeout:", drive_timeout)
        drivetrain.set_timeout(drive_timeout, SECONDS)
        start_time = drive_timer.system_high_res()
        drivetrain.drive_straight_for(FORWARD, distance, MM, heading=heading)
        end_time = drive_timer.system_high_res()
        drive_time = (end_time - start_time) / 1000000
        #print("drive_time:", drive_time)
        #print_tracker(tracker, x, y)
    
        # do action

        if i == 2:
            mid_goal_score()

        if i == 3:
            close_trapdoor()
            run_intake(True, use_volt=True)
            lower_flippy()

        if i == 4:
            time_to_wait = 2.0 - drive_time
            wait(time_to_wait, SECONDS)
        
        if i == 5:
            enable_color_sort(True)
            run_shooter(True)
            time_limit = False
            while not detect_color_both_sensors(reject_color) and not time_limit:
                wait(10, MSEC)
                seq_stop_time = drive_timer.time()
                seq_time = (seq_stop_time - seq_start_time) / 1000.0
                if seq_time > 15.0:
                    pass
                    # time_limit = True
            wait(0.2, SECONDS)
            stop_shooter()
            stop_intake()
            enable_color_sort(False)

        i += 1

    print_tracker(tracker)

def auton_match_right(tracker: Tracking):
    print("auton_match_right")
    brain.screen.print("auton_match_right")
    brain.screen.new_line()

    reject_color = DetectColor.RED if ALLIANCE_COLOR == AllianceColor.BLUE else DetectColor.BLUE

    setup_drivetrain()
    drive_speed = 50 # PERCENT
    turn_speed = 40 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(600, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)
    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_turn_velocity(turn_speed, PERCENT)

    drivetrain.set_stopping(BrakeType.BRAKE)

    
    drive_timer = Timer()
    seq_start_time = drive_timer.time()

    print_tracker(tracker)


    x_start = 600.0 - (16.25 - 2.5) * 25.4
    y_start = 3600.0 - (1200.0 + (6.875 + 0.375) * 25.4)

    points = [
        [x_start, y_start, False], # start point
        [900.0, y_start, False], # 0

        [1250.0, 3600.0 - 1250.0, False], # 1: pick up 3 balls

        [610.0, 3600.0 - 590.0, True], # 2: Align with hopper
        [1200.0-95.0+20.0, 3600.0 - 590.0, True], # 3: score long goal

        [385.0, 3600.0 - 600.0, False], # 4: hopper 600 - 6.75 * 25.4, dot on
        [1200.0-95.0+20.0, 3600.0 - 590.0, True] # 5: score long goal
        
    ]
    start_point = points.pop(0)
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    run_intake(True, use_volt=True)

    i = 0 
    for point in points:
        x = point[0]
        y = point[1]
        rev = point[2]

       # pre actions
        turn_timeout = 1.0 # HACK

        if i == 0:
            drivetrain.set_drive_threshold(10.0) # MM

        if i == 1:
            drivetrain.set_drive_threshold(5.0) # MM
            turn_timeout = 0.25

        # Turn phase
        # no turn coming out of parking
        if i > 0:
            distance, heading = tracker.trajectory_to_point(x=x, y=y, reverse=rev)
            drivetrain.set_timeout(turn_timeout, SECONDS)
            drivetrain.turn_to_heading(heading)
            #print_tracker(tracker, x, y)

        # Drive phase
        distance, heading = tracker.trajectory_to_point(x=x, y=y, reverse=rev)
        drive_timeout = 1.0 + abs(distance / linear_speed_mm_sec) # convert to MM/s and pad with 1 sec
        #print("drive_timeout:", drive_timeout)
        drivetrain.set_timeout(drive_timeout, SECONDS)
        start_time = drive_timer.system_high_res()
        drivetrain.drive_straight_for(FORWARD, distance, MM, heading=heading)
        end_time = drive_timer.system_high_res()
        drive_time = (end_time - start_time) / 1000000
        print("drive_time:", drive_time)
        #print_tracker(tracker, x, y)
    
        # do action

        if i == 3:
            run_shooter(True)
            wait(2.0, SECONDS)
            stop_shooter()
            lower_flippy()

        if i == 4:
            time_to_wait = 2.0  - drive_time
            wait(time_to_wait, SECONDS)
        
        if i == 5:
            enable_color_sort(True)
            run_shooter(True)
            time_limit = False
            while not detect_color_both_sensors(reject_color) and not time_limit:
                wait(10, MSEC)
                seq_stop_time = drive_timer.time()
                seq_time = (seq_stop_time - seq_start_time) / 1000.0
                if seq_time > 14.5:
                    pass
                    # time_limit = True

            seq_stop_time = drive_timer.time()
            print("Totoal Time", (seq_stop_time - seq_start_time) / 1000.0)
            wait(0.2, SECONDS)
            stop_shooter()
            stop_intake()
            enable_color_sort(False)

        i += 1
        
        print_tracker(tracker)

def auton_match_none():
    print("auton_match_none")
    brain.screen.print("auton_match_none")
    brain.screen.new_line()
    # gear ratio over circumference
    # for each rev of the motor mutliply by the grear ratio times circumfrence    
    distance_per_rev = DRIVETRAIN_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE # this is going to be in MM
    
    left_motor_group.spin_for(FORWARD, 6.0 * 25.4 /distance_per_rev, RotationUnits.REV, 25, PERCENT, wait=False)
    right_motor_group.spin_for(FORWARD, 6.0 * 25.4/ distance_per_rev, RotationUnits.REV, 25, PERCENT, wait=False)
    wait(1,SECONDS)

    left_motor_group.stop(COAST)
    right_motor_group.stop(COAST)

def hopper_drain2():
    if (tracker is None):
        return
    
    distance = 100.0
    wheel_turns = distance / (DRIVETRAIN_WHEEL_SIZE * DRIVETRAIN_GEAR_RATIO)
    left_motor_group.spin_for(REVERSE, wheel_turns, RotationUnits.REV, 50, PERCENT, wait=False)
    right_motor_group.spin_for(REVERSE, wheel_turns, RotationUnits.REV, 50, PERCENT, wait=True)
    wait(0.25, SECONDS)
    wheel_turns = distance / (DRIVETRAIN_WHEEL_SIZE * DRIVETRAIN_GEAR_RATIO)
    left_motor_group.spin_for(FORWARD, wheel_turns, RotationUnits.REV, 50, PERCENT, wait=False)
    right_motor_group.spin_for(FORWARD, wheel_turns, RotationUnits.REV, 50, PERCENT, wait=True)

    while True: 
        wait(0.25, SECONDS)


def hopper_drain():
    if (tracker is None):
        return
    
    drivetrain.set_drive_velocity(50, PERCENT)


    location = tracker.get_orientation()
    start_x = location.x
    target_x = 300.0
    distance = abs(target_x - start_x)
    drivetrain.drive_straight_for(FORWARD, distance, MM, heading=180.0)
    
    for i in range(3):
        wait(0.5, SECONDS)
        location = tracker.get_orientation()
        start_x = location.x
        target_x = 350.0
        distance = abs(target_x - start_x)
        drivetrain.drive_straight_for(REVERSE, distance, MM, heading=180.0)

        location = tracker.get_orientation()
        start_x = location.x
        target_x = 400.0
        distance = abs(target_x - start_x)
        drivetrain.drive_straight_for(FORWARD, distance, MM, heading=180.0)


def auton_skills(tracker: Tracking):
    print("auton_skills")
    brain.screen.print("auton_match_right")
    brain.screen.new_line()

    reject_color = DetectColor.BLUE

    setup_drivetrain()
    drive_speed = 33 # PERCENT
    turn_speed = 50 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(600, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)
    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_turn_velocity(turn_speed, PERCENT)

    drivetrain.set_stopping(BrakeType.BRAKE)

    drive_timer = Timer()
    seq_start_time = drive_timer.time()

    print_tracker(tracker)

    x_start = 600.0 - (14.5) * 25.4 # check
    y_start = 3600 - (1200.0 + 5.5 * 25.4) # check

    points = [
        [x_start, y_start, False], # start point
        [850.0, y_start, False], # 0

        [1200.0, 3600.0 - 1200.0, False], # 1: pick up 3 balls

        [610.0, 3600.0 - 590.0, True], # 2: Align with hopper
        [1200.0-95.0+10.0, 3600.0 - 590.0, True], # 3: score long goal

        [400.0, 3600.0 - 600.0, False], # 4: hopper 600 - 6.75 * 25.4, dot on
        [1200.0-95.0 + 10.0, 3600.0 - 590.0, True], # 5: score long goal

        #starting left sequence
        [610.0, 3600.0 - 590.0, False], # 6: Align with hopper

        [900.0, 1800.0, False], # 7: Fake drive to match start point

        [1200.0, 1200.0, False], # 8: pick up 3 balls
        
        [610.0, 590.0, True], # 9: Align with hopper
        [1200.0-95.0+20.0, 590.0, True], # 10: score long goal

        [385.0, 600.0, False], # 11: hopper 600 - 6.75 * 25.4, dot on
        [1200.0-95.0+30.0, 590.0, True], # 12: score long goal
        
    ]
    start_point = points.pop(0)
    tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    run_intake(True, use_volt=True)

    i = 0 
    for point in points:
        x = point[0]
        y = point[1]
        rev = point[2]
        print("----- MOVE", i, x, y)

        # pre actions
        turn_timeout = 1.0 # HACK

        if i == 0:
            drivetrain.set_drive_threshold(10.0) # MM

        if i == 1:
            drivetrain.set_drive_threshold(5.0) # MM
            turn_timeout = 0.25

        # Turn phase
        # no turn coming out of parking
        if i > 0:
            distance, heading = tracker.trajectory_to_point(x=x, y=y, reverse=rev)
            drivetrain.set_timeout(turn_timeout, SECONDS)
            drivetrain.turn_to_heading(heading)
            #print_tracker(tracker, x, y)

        # Drive phase
        distance, heading = tracker.trajectory_to_point(x=x, y=y, reverse=rev)
        drive_timeout = 1.0 + abs(distance / linear_speed_mm_sec) # convert to MM/s and pad with 1 sec
        #print("drive_timeout:", drive_timeout)
        drivetrain.set_timeout(drive_timeout, SECONDS)
        start_time = drive_timer.system_high_res()
        drivetrain.drive_straight_for(FORWARD, distance, MM, heading=heading)
        end_time = drive_timer.system_high_res()
        drive_time = (end_time - start_time) / 1000000
        print("drive_time:", drive_time)
        print_tracker(tracker, x, y)
    
        # do action

        if i == 3:
            run_shooter(True)
            wait(5.0, SECONDS)
            stop_shooter()
            lower_flippy()

        if i == 4:
            hopper_drain2()
            wait(3.0, SECONDS)
        
        if i == 5:
            run_shooter(True)
            wait(6.0, SECONDS)
            stop_shooter()
            stop_intake()
            raise_flippy()
            # tracker.set_orientation(Tracking.Orientation(x=1200 - 80.0, y=3600 - 600.0, heading=None))

        if i ==6:
            run_intake(True)
        
        if i == 10:
            run_shooter(True)
            wait(5.0, SECONDS)
            stop_shooter()
            lower_flippy()
        
        if i == 11:
            wait(3.0, SECONDS)

        if i == 12:
            run_shooter(True)
            wait(6.0, SECONDS)
            stop_shooter()
            stop_intake()
            raise_flippy()

        i += 1

        print_tracker(tracker)

def OnLoggerPositionCallback():
    if tracker is None:
        raise Exception("tracker not initialized")
    return tracker.x, tracker.y # type: ignore

def auton_test_turns(tracker: Tracking):

    print("auton_test_turns")
    brain.screen.print("auton_test_turns")
    brain.screen.new_line()

    setup_drivetrain()
    drive_speed = 50 # PERCENT
    turn_speed = 50 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(600, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)
    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_turn_velocity(turn_speed, PERCENT)

    drivetrain.set_stopping(BrakeType.BRAKE)

    drive_timer = Timer()
    seq_start_time = drive_timer.time()

    tracker.enable(False)
    wait(50, MSEC)
    tracker.set_orientation(Tracking.Orientation(0.0, 0.0, 45.0))
    wait(50, MSEC)
    tracker.enable(True)
    wait(50, MSEC)

    log = Logger(brain,
                 [left_motor_group, right_motor_group] + all_sensors,
                 ["lmg", "rmg", "gyro", "fwd", "side"],
                 ["x", "y"], OnLoggerPositionCallback,
                 time_sec = 20,
                 auto_dump = True, file_name = "auton_test_turns"
                )
    log.start()

    print_tracker(tracker)

    wait (100, MSEC)
    drivetrain.turn_to_heading(180, DEGREES)

    wait (500, MSEC)

    print_tracker(tracker)
    #drivetrain.turn_for(RIGHT, 135, DEGREES)

    #wait (500, MSEC)

    #print_tracker(tracker)
    log.stop(True)

    print("auton done")


def autonomous():
    global ROBOT_IS_ENABLED
    ROBOT_IS_ENABLED = True
    # wait for initialization code
    while (not initialization_complete or tests_running):
        wait(10, MSEC)

    print("autonomous code")
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Alliance color", "RED" if ALLIANCE_COLOR == AllianceColor.RED else "BLUE")
    brain.screen.new_line()
    brain.screen.print("Sequence:", AUTON_SEQUENCE)
    brain.screen.new_line()

    if (inertial_sensor.installed):
        brain.screen.print("GYRO OK")
        brain.screen.new_line()
    else:
        brain.screen.print("GYRO MISSING")
        brain.screen.new_line()

    if tracker is None:
        raise RuntimeError("Tracker not initialized")
    tracker.enable()

    wait(10, MSEC)
    print_tracker(tracker)

    # place automonous code here 
    # auto1()
    # auto2()
    # auto3(tracker)
    # auto4_drive_to_points(tracker)    
    # auto4_match(tracker)
    # auton_match_left(tracker)
    # auton_match_right(tracker)

    if AUTON_SEQUENCE == AutonSequence.SKILLS:
        auton_test_turns(tracker)
        # auton_skills(tracker)
    elif AUTON_SEQUENCE == AutonSequence.MATCH_LEFT:
        auton_match_left(tracker)
    elif AUTON_SEQUENCE == AutonSequence.MATCH_RIGHT:
        auton_match_right(tracker)
    elif AUTON_SEQUENCE == AutonSequence.MATCH_NONE:
        auton_match_none()

    print_tracker(tracker)
    
# ------------------------------------------------------------ #
### DRIVER CONTROL

# Controller mapping - note there are two control modes:
# - individual control uses all four bumpers L1, L2, R1, R2 and controls intake and shooter independently
# - linked control mapped as follors:
#   - R1: intake
#   - R2: eject
#   - L1: long goal score (intake + shooter)
#   - L2: mid-level goal score (intake + trapdoor open)
# All buttons are toggles in both modes
control_mode = 1 # 0 is individual control, 1 is linked control
# Other functions mapped as follows
# - A: Lower/Raise intake flip mech
# - B: Open/Close trapdoor
# - X: Enable/Disable color sort (currently only rejects blue)
# - Up: Enable/Disable drive straight (driver control). Uses inertial sensor to keep robot on straight path
# - Down: Enable/Disable full speed mode (driver control). By default 480RPM drive is maade to look like 240RPM drive

# Intake
def OnButtonR1Pressed():
    close_trapdoor()
    if control_mode == 0:
        if ramp_speed == 0: run_intake(True)
        else: stop_intake()
    else:
        stop_shooter()
        if ramp_speed == 0:
            run_intake(True)
        elif ramp_speed == RAMP_MAX_INTAKE:
            stop_intake()
        elif ramp_speed == RAMP_MAX_EJECT:
            stop_intake()
            wait(0.1, SECONDS)
            run_intake(True)

# Eject
def OnButtonR2Pressed():
    close_trapdoor()
    if control_mode == 0:
        if ramp_speed == 0: run_intake(False)
        else: stop_intake()
    else:
        stop_shooter()
        if ramp_speed == 0:
            run_intake(False)
        elif ramp_speed < 0:
            stop_intake()
        elif ramp_speed > 0:
            stop_intake()
            wait(0.1, SECONDS)
            run_intake(False)

# Score
def OnButtonL1Pressed():
    close_trapdoor()
    if control_mode == 0:
        if (shooter_speed == 0): run_shooter(True)
        else: stop_shooter()
    else:
        if ramp_speed == 0:
            run_intake(True)
            run_shooter(True)
        elif ramp_speed == RAMP_MAX_INTAKE and shooter_speed != 0:
            stop_intake()
            stop_shooter()
        elif ramp_speed == RAMP_MAX_INTAKE and shooter_speed == 0:
            run_intake(True)
            run_shooter(True)
        elif ramp_speed == RAMP_MAX_EJECT:
            stop_intake()
            wait(0.1, SECONDS)
            run_intake(True)
            run_shooter(True)

# Mid level score
def OnButtonL2Pressed():
    if control_mode == 0:
        if (shooter_speed == 0): run_shooter(False)
        else: stop_shooter()
    else:
        stop_shooter()
        if (ramp_speed == 0):
            open_trapdoor()
            run_intake(True)
            run_shooter(False)
        elif (ramp_speed == RAMP_MAX_INTAKE):
            if (not trapdoor_is_open()):
                open_trapdoor()
                run_shooter(False)
            else:
                stop_intake()
                stop_shooter()
                close_trapdoor()
        elif ramp_speed == RAMP_MAX_EJECT:
            open_trapdoor()
            stop_intake()
            wait(0.1, SECONDS)
            run_intake(True)
            run_shooter(False)

# Drive straight enable / disable
ENABLE_DRIVE_STRAIGHT = False
def OnButtonUpPressed():
    global ENABLE_DRIVE_STRAIGHT
    if (ENABLE_DRIVE_STRAIGHT): ENABLE_DRIVE_STRAIGHT = False
    else: ENABLE_DRIVE_STRAIGHT = True

# Full speed toggle
ENABLE_FULL_SPEED = False
FULL_SPEED = 100
SLOW_SPEED = 50
CURRENT_SPEED = SLOW_SPEED
def OnButtonDownPressed():
    global ENABLE_FULL_SPEED, CURRENT_SPEED
    if (ENABLE_FULL_SPEED): ENABLE_FULL_SPEED = False
    else: ENABLE_FULL_SPEED = True
    CURRENT_SPEED = FULL_SPEED if ENABLE_FULL_SPEED else SLOW_SPEED

# Flippy solenoid toggle
def OnButtonAPressed():
    if flippy_is_lowered():
        raise_flippy()
    else:
        lower_flippy()

# Trapdoor solenoid toggle
def OnButtonBPressed():
    if trapdoor_is_open():
        close_trapdoor()
    else:
        open_trapdoor()

# Color sort toggle
color_sort_enabled = False
def OnButtonLeftPressed():
    global color_sort_enabled
    if color_sort_enabled:
        print("color sort off")
        enable_color_sort(False)
        color_sort_enabled = False
    else:
        print("color sort on")
        enable_color_sort(True)
        color_sort_enabled = True

def user_control():
    global ROBOT_IS_ENABLED
    ROBOT_IS_ENABLED = True
    # wait for initialization code
    while (not initialization_complete or tests_running):
        wait(10, MSEC)

    if tracker is None:
        raise RuntimeError("Tracker not initialized")
    tracker.enable()
    # tracker.set_orientation(Tracking.Orientation(start_point[0], start_point[1], 0.0))

    # clean-up from auton
    enable_color_sort(False)
    stop_intake()
    stop_shooter()

    print("driver control")
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Driver control")
    brain.screen.new_line()
    brain.screen.print("Alliance color", "RED" if ALLIANCE_COLOR == AllianceColor.RED else "BLUE")
    brain.screen.new_line()

    # events
    controller_1.buttonR1.pressed(OnButtonR1Pressed) # Intake
    controller_1.buttonR2.pressed(OnButtonR2Pressed) # Eject
    controller_1.buttonL1.pressed(OnButtonL1Pressed) # Shooter On or Long Goal Score
    controller_1.buttonL2.pressed(OnButtonL2Pressed) # Shooter Reverse or Mid Level Goal Score
    controller_1.buttonUp.pressed(OnButtonUpPressed) # Enable / disable drive straight - DEMO ONLY
    controller_1.buttonDown.pressed(OnButtonDownPressed) # Toggle full vs reduced speed
    controller_1.buttonA.pressed(OnButtonAPressed) # Toggle arm solenoid
    controller_1.buttonB.pressed(OnButtonBPressed) # Toggle trapdoor solenoid
    controller_1.buttonX.pressed(OnButtonLeftPressed) # Toggle color sort

    # drivetrain control
    driver_control = DriverControl(left_motor_group, right_motor_group, inertial_sensor)
    driver_control.set_mode(follow_heading_Kp=2.0)

    # place driver control in this while loop
    loop_count = 0
    while True:
        if (loop_count % 200) == 0:
            print_tracker(tracker)
        loop_count += 1

        driver_control.set_mode(enable_drive_straight=ENABLE_DRIVE_STRAIGHT)
        driver_control.set_speed_limits(drive_max=CURRENT_SPEED)
        driver_control.user_drivetrain(controller_1.axis3.position(), controller_1.axis1.position())
        if color_sort_enabled and shooter_speed != 0:
            # stop_on_color()
            stop_on_color()
        wait(10, MSEC) # DO NOT CHANGE

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
pre_autonomous()

