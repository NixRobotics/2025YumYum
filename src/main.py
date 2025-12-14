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
from v5pythonlibrary import *

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
DRIVETRAIN_WHEEL_SIZE = 260.0 # Wheel circumherence in MM
DRIVETRAIN_TRACK_WIDTH = 320.0 # Not used
DRIVETRAIN_WHEEL_BASE = 320.0 # Not used
DRIVETRAIN_GEAR_RATIO = 36.0/48.0

ramp_motor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
shooter_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)

GYRO_SCALE = (360.0 + 3.475) / 360.0# this is roughly how much robot over-turns per revolution
inertial_sensor = InertialWrapper(Ports.PORT16, GYRO_SCALE)

flippy_solenoid = DigitalOut(brain.three_wire_port.h)
ramp_solenoid = DigitalOut(brain.three_wire_port.g)

color1 = Optical(Ports.PORT6)
color2 = Optical(Ports.PORT7)

# vex helper constructs
left_motor_group = MotorGroup(left_front_motor, left_mid_motor, left_back_motor)
right_motor_group = MotorGroup(right_front_motor, right_mid_motor, right_back_motor)
drivetrain = SmartDriveWrapper(left_motor_group, right_motor_group, inertial_sensor,
                               DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_TRACK_WIDTH, DRIVETRAIN_WHEEL_BASE,
                               DistanceUnits.MM,
                               DRIVETRAIN_GEAR_RATIO)

# ------------------------------------------------------------ #
# Motor Monitoring
class MotorMonitor:
    global left_front_motor, left_mid_motor, left_back_motor, right_front_motor, right_mid_motor, right_back_motor, ramp_motor, shooter_motor

    # motors will slow down above certain temperatures
    # vex defines warm as 50 percent and hot as 70 percent
    MOTOR_HOT_TEMP = 70
    MOTOR_WARM_TEMP = 60
    MOTOR_COOL_TEMP = 50

    MOTOR_STATUS_OK = 0
    MOTOR_STATUS_NOT_PRESENT = 1
    MOTOR_STATUS_COOL = 2
    MOTOR_STATUS_WARM = 3
    MOTOR_STATUS_HOT = 4

    def __init__(self):
        self.allmotors = [
            left_front_motor, left_mid_motor, left_back_motor,
            right_front_motor, right_mid_motor, right_back_motor,
            ramp_motor, shooter_motor]
        self.motor_names = [
            "Left Front", "Left Mid", "Left Back",
            "Right Front", "Right Mid", "Right Back",
            "Ramp", "Shooter"]
        self.motor_status = [
            MotorMonitor.MOTOR_STATUS_OK, MotorMonitor.MOTOR_STATUS_OK, MotorMonitor.MOTOR_STATUS_OK,
            MotorMonitor.MOTOR_STATUS_OK, MotorMonitor.MOTOR_STATUS_OK, MotorMonitor.MOTOR_STATUS_OK,
            MotorMonitor.MOTOR_STATUS_OK, MotorMonitor.MOTOR_STATUS_OK]
        self.previous_motor_status = self.motor_status.copy()
        self.motor_status_names = ["OK", "NOT PRESENT", "OK FOR NOW", "WARM", "HOT"]

    # this function checks the temperature of each motor and then returns two values: warm, hot
    def get_motor_status(self):
        # to make code simple we create a list of all the motors, then check each one
        motor_error = False
        motor_is_cool = False
        motor_is_warm = False
        motor_is_hot = False
        for i in range(len(self.allmotors)):
            motor = self.allmotors[i]
            self.motor_status[i] = MotorMonitor.MOTOR_STATUS_OK # assume motor is ok
            if (not motor.installed()):
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_NOT_PRESENT
                motor_error = True
            elif motor.temperature(PERCENT) >= MotorMonitor.MOTOR_HOT_TEMP:
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_HOT
                motor_is_hot = True
            elif motor.temperature(PERCENT) >= MotorMonitor.MOTOR_WARM_TEMP:
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_WARM
                motor_is_warm = True
            elif motor.temperature(PERCENT) >= MotorMonitor.MOTOR_COOL_TEMP:
                self.motor_status[i] = MotorMonitor.MOTOR_STATUS_COOL
                motor_is_cool = True

        is_changed = self.motor_status != self.previous_motor_status
        self.previous_motor_status = self.motor_status.copy()

        return is_changed, motor_error, motor_is_cool, motor_is_warm, motor_is_hot

    # we run this in its own thread to monitor the temperature each second and change the color of the screen
    # green is first warning
    # blue is warm
    # red is hot
    def monitor_UI(self, motor_error, motor_is_cool, motor_is_warm, motor_is_hot):

        if (motor_is_hot): brain.screen.clear_screen(Color.RED)
        elif (motor_is_warm): brain.screen.clear_screen(Color.BLUE)
        elif (motor_is_cool): brain.screen.clear_screen(Color.GREEN)
        else: brain.screen.clear_screen(Color.BLACK)

        if (motor_error or motor_is_hot or motor_is_warm or motor_is_cool):
            brain.screen.set_cursor(1,1)
            brain.screen.set_font(FontType.MONO20)
            brain.screen.set_fill_color(Color.BLACK)
            brain.screen.set_pen_color(Color.WHITE)
            for i in range(len(self.allmotors)):
                if (self.motor_status[i] != MotorMonitor.MOTOR_STATUS_OK):
                    brain.screen.print(self.motor_names[i] + ": " + self.motor_status_names[self.motor_status[i]])
                    brain.screen.new_line()
            # brain.screen.render()

    @staticmethod
    def motor_monitor_thread():
        monitor = MotorMonitor()
        while(True):
            is_changed, motor_error, motor_is_cool, motor_is_warm, motor_is_hot = monitor.get_motor_status()
            # only call UI routines if status has changed
            if (is_changed):
                monitor.monitor_UI(motor_error, motor_is_cool, motor_is_warm, motor_is_hot)
            wait(2, TimeUnits.SECONDS)

# ------------------------------------------------------------ #
# ROBOT POSITION TRACKING

USING_TRACKING_WHEELS = False
USING_RESAMPLING = False

if USING_TRACKING_WHEELS:
    rotation_fwd = Rotation(Ports.PORT6, False)
    rotation_strafe = Rotation(Ports.PORT7, False)
    all_sensors = [inertial_sensor, rotation_fwd, rotation_strafe]

    ODOMETRY_FWD_SIZE = 218.344
    ODOMETRY_FWD_OFFSET = 0.0316 * 25.4
    ODOMETRY_FWD_GEAR_RATIO = 1.0
    ODOMETRY_STRAFE_SIZE = 157.38
    ODOMETRY_STRAFE_OFFSET = 4.526 * 25.4
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

    tracker = Tracking(tracker_devices, starting_location, tracker_configuration, initial_values=None)
    tracker.enable_resampling(USING_RESAMPLING)
    # give tracker some time to get going
    wait(0.1, SECONDS)

def print_tracker(tracker: Tracking, x = 0.0, y = 0.0):
    orientation = tracker.get_orientation()
    origin_distance, origin_heading = tracker.trajectory_to_point(x, y)
    print("X: {:.1f} mm, Y: {:.1f} mm, Heading: {:.2f} deg".format(orientation.x, orientation.y, orientation.heading))
    print(" - To Point: Distance: {:.1f} mm, Heading: {:.2f} deg".format(origin_distance, origin_heading))

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
def run_intake(bIntake):
    global ramp_speed
    if ((bIntake and ramp_speed == RAMP_MAX_EJECT) or (not bIntake and ramp_speed == RAMP_MAX_INTAKE)):
        print("reversing intake")
        stop_intake()
        wait(0.1, SECONDS) # wait for intake to slow before reversing direction
    if bIntake: ramp_speed = RAMP_MAX_INTAKE
    else: ramp_speed = RAMP_MAX_EJECT
    ramp_motor.spin(REVERSE, ramp_speed, PERCENT)

def stop_intake():
    global ramp_speed
    ramp_speed = 0
    ramp_motor.stop(COAST)

def run_shooter(bUp):
    global shooter_speed
    if (shooter_speed != 0):
        stop_shooter()
        wait(0.1, SECONDS) # wait for conveyor to slow before reversing direction
    if bUp: shooter_speed = SHOOTER_MAX_UP
    else: shooter_speed = SHOOTER_MAX_DOWN
    shooter_motor.spin(FORWARD, shooter_speed, PERCENT)

def stop_shooter():
    global shooter_speed
    shooter_speed = 0
    shooter_motor.stop(COAST)

# trapdoor control functions
def open_trapdoor():
    ramp_solenoid.set(1)

def close_trapdoor():
    ramp_solenoid.set(0)

def trapdoor_is_open():
    return ramp_solenoid.value() == 1

# flippy control functions
def raise_flippy():
    flippy_solenoid.set(0)

def lower_flippy():
    flippy_solenoid.set(1)

def flippy_is_lowered():
    return flippy_solenoid.value() == 1

# ------------------------------------------------------------ #
# Expermimental color sort function - only scores red right now (rejects blue)

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
DETECT_BLUE = 0
DETECT_RED = 1

def detect_color(sensor: Optical, color: int):
    '''
    ### Docstring for detect_color
    
    :param sensor: Optical sensor object
    :type sensor: Optical
    :param color: color to detect (DETECT_BLUE or DETECT_RED)
    :type color: int
    '''
    if color == DETECT_BLUE:
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

color_sort_valid = False
color_sort_valid_count = 0
color_sort_shooter_speed = 0

def color_sort():
    global color_sort_valid, color_sort_valid_count, color_sort_shooter_speed
    if detect_color(color1, DETECT_BLUE) or detect_color(color2, DETECT_BLUE):
        if not color_sort_valid:
            ramp_solenoid.set(1)
            color_sort_shooter_speed = shooter_speed
            shooter_motor.stop(COAST)
            print("Blue detected")
        color_sort_valid = True
        color_sort_valid_count = 0

    elif color_sort_valid:
        if color_sort_valid_count < 15:
            color_sort_valid_count += 1
        else:
            ramp_solenoid.set(0)
            shooter_motor.spin(FORWARD, color_sort_shooter_speed, PERCENT)
            color_sort_valid = False
            print("Blue lost")

def stop_on_color():
    global color_sort_valid, color_sort_valid_count, color_sort_shooter_speed
    if detect_color(color1, DETECT_BLUE) or detect_color(color2, DETECT_BLUE):
        stop_intake()
        stop_shooter()

# ------------------------------------------------------------ #
### AUTONOMOUS ###

ROBOT_IS_ENABLED = False

def pre_auton_UI():
    mode = 0
    brain.screen.clear_screen()
    brain.screen.set_fill_color(Color.BLACK)
    brain.screen.draw_rectangle(10, 10, 100, 50)
    while not ROBOT_IS_ENABLED:
        if brain.screen.pressing():
            mode += 1
            if mode > 5:
                mode = 0
            brain.screen.clear_screen()
            if mode == 0:
                brain.screen.set_fill_color(Color.BLACK)
                brain.screen.draw_rectangle(10, 10, 100, 50)
            elif mode == 1:
                brain.screen.set_fill_color(Color.GREEN)
                brain.screen.draw_rectangle(300, 10, 100, 50)
            elif mode == 2:
                brain.screen.set_fill_color(Color.RED)
                brain.screen.draw_rectangle(10, 75, 100, 50)
            elif mode == 3:
                brain.screen.set_fill_color(Color.RED)
                brain.screen.draw_rectangle(300, 75, 100, 50)
            elif mode == 4:
                brain.screen.set_fill_color(Color.BLUE)
                brain.screen.draw_rectangle(10, 150, 100, 50)
            elif mode == 5:
                brain.screen.set_fill_color(Color.BLUE)
                brain.screen.draw_rectangle(300, 150, 100, 50)
            wait(0.5, SECONDS)
        wait(0.1, SECONDS)

ALLIANCE_COLOR = "RED" # "RED" or "BLUE"

def pre_autonomous():
    global initialization_complete, ALLIANCE_COLOR
    # actions to do when the program starts
    # wait a bit before doing anything to let devices initialize
    print("pre-auton code")

    wait(0.1, SECONDS)
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("pre auton code")

    # calibrate the inertial sensor
    if (inertial_sensor.installed):
        inertial_sensor.calibrate()
        while inertial_sensor.is_calibrating():
            wait(10, TimeUnits.MSEC)

    # start background threads
    motor_monitor_thread = Thread(MotorMonitor.motor_monitor_thread)
    initialize_tracker()
    enable_color_sort(True)

    wait(0.1, SECONDS)

    print("a piston: ", flippy_solenoid.value())
    print("b piston: ", ramp_solenoid.value())
    if detect_color(color1, DETECT_BLUE) or detect_color(color2, DETECT_BLUE):
        print("BLUE")
        brain.screen.set_cursor(2,1)
        brain.screen.print("BLUE DETECTED")
        ALLIANCE_COLOR = "BLUE" # default to red if blue detected

    enable_color_sort(False)
    initialization_complete = True

    pre_auton_UI()

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

def auto1():
    print("auto1")
    # gear ratio over circumference
    # for each rev of the motor mutliply by the grear ratio times circumfrence    
    distance_per_rev = DRIVETRAIN_GEAR_RATIO * DRIVETRAIN_WHEEL_SIZE # this is going to be in MM
    
    left_motor_group.spin_for(FORWARD, 6.0 * 25.4 /distance_per_rev, RotationUnits.REV, 25, PERCENT, wait=False)
    right_motor_group.spin_for(FORWARD, 6.0 * 25.4/ distance_per_rev, RotationUnits.REV, 25, PERCENT, wait=False)
    wait(1,SECONDS)

    left_motor_group.stop(COAST)
    right_motor_group.stop(COAST)

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
    drivetrain.set_drive_accleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_headling_lock_constants(5.0)

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
    drivetrain.set_drive_accleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_drive_threshold(5) # MM

    drivetrain.set_turn_velocity(turn_speed, PERCENT)
    drivetrain.set_turn_constants(Kp=3.0, Ki=0.06, Kd=15.0)
    drivetrain.set_turn_threshold(0.5) # DEGREES

    drivetrain.set_headling_lock_constants(Kp=5.0)

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
    drivetrain.set_drive_accleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_drive_threshold(5) # MM

    drivetrain.set_turn_velocity(turn_speed, PERCENT)
    drivetrain.set_turn_constants(Kp=3.0, Ki=0.06, Kd=15.0)
    drivetrain.set_turn_threshold(0.5) # DEGREES

    drivetrain.set_headling_lock_constants(Kp=4.0)

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

def auton5(tracker: Tracking):
    print("auton5")

    drive_speed = 50 # PERCENT
    turn_speed = 40 # PERCENT
    linear_speed_mm_sec, turn_speed_rev_sec = drivetrain_max_speeds(600, DRIVETRAIN_WHEEL_SIZE, DRIVETRAIN_GEAR_RATIO)
    linear_speed_mm_sec *= (drive_speed / 100)
    turn_speed_rev_sec *= (turn_speed / 100)

    drivetrain.set_drive_velocity(drive_speed, PERCENT)
    drivetrain.set_drive_accleration(3, PERCENT)
    drivetrain.set_drive_constants(0.2, Ki=0.002, Kd=1)
    drivetrain.set_drive_threshold(5) # MM

    drivetrain.set_turn_velocity(turn_speed, PERCENT)
    drivetrain.set_turn_constants(Kp=3.0, Ki=0.06, Kd=15.0)
    drivetrain.set_turn_threshold(1) # DEGREES

    drivetrain.set_headling_lock_constants(Kp=5.0)

    drivetrain.set_stopping(BrakeType.BRAKE)

    print_tracker(tracker)

    distance, heading = tracker.trajectory_to_point(x=100, y=0, reverse=False)
    drivetrain.drive_straight_for(FORWARD, 100.0, MM, heading=0.0)
    drivetrain.turn_to_heading(90)
    drivetrain.turn_to_heading(0)
    drivetrain.drive_straight_for(REVERSE, 100.0, MM, heading=0.0)

    print_tracker(tracker)

def autonomous():
    global ROBOT_IS_ENABLED
    ROBOT_IS_ENABLED = True
    # wait for initialization code
    while (not initialization_complete or tests_running):
        wait(10, MSEC)

    print("autonomous code")
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("autonomous code", ALLIANCE_COLOR)
    brain.screen.new_line()
    if (inertial_sensor.installed):
        brain.screen.print("GYRO OK")
    else:
        brain.screen.print("GYRO MISSING")

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
    auton5(tracker)

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
        elif (ramp_speed == RAMP_MAX_INTAKE):
            if (not trapdoor_is_open()):
                open_trapdoor()
            else:
                stop_intake()
                close_trapdoor()
        elif ramp_speed == RAMP_MAX_EJECT:
            open_trapdoor()
            stop_intake()
            wait(0.1, SECONDS)
            run_intake(True)

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
def OnButtonXPressed():
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

    print("driver control")
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("driver control", ALLIANCE_COLOR)

    # events
    controller_1.buttonR1.pressed(OnButtonR1Pressed) # Intake
    controller_1.buttonR2.pressed(OnButtonR2Pressed) # Eject
    controller_1.buttonL1.pressed(OnButtonL1Pressed) # Shooter On or Long Goal Score
    controller_1.buttonL2.pressed(OnButtonL2Pressed) # Shooter Reverse or Mid Level Goal Score
    controller_1.buttonUp.pressed(OnButtonUpPressed) # Enable / disable drive straight - DEMO ONLY
    controller_1.buttonDown.pressed(OnButtonDownPressed) # Toggle full vs reduced speed
    controller_1.buttonA.pressed(OnButtonAPressed) # Toggle arm solenoid
    controller_1.buttonB.pressed(OnButtonBPressed) # Toggle trapdoor solenoid
    controller_1.buttonX.pressed(OnButtonXPressed) # Toggle color sort

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
            color_sort()
        wait(10, MSEC) # DO NOT CHANGE

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
pre_autonomous()

