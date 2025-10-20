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

# declare devices
brain = Brain()
controller_1 = Controller(PRIMARY)

initialization_complete = False
tests_running = False

left_front_motor = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
left_mid_motor = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
left_back_motor = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)

right_front_motor = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False)
right_mid_motor = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
right_back_motor = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)

ramp_motor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
shooter_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)

inertial_sensor = Inertial(Ports.PORT2)
GYRO_SCALE = 360.0 / (360.0 + 0.0) # this is roughly how much robot over-turns per revolution

arm_solenoid = DigitalOut(brain.three_wire_port.h)
ramp_solenoid = DigitalOut(brain.three_wire_port.g)

color1 = Optical(Ports.PORT6)
color2 = Optical(Ports.PORT7)

# vex helper constructs
left_motor_group = MotorGroup(left_front_motor, left_mid_motor, left_back_motor)
right_motor_group = MotorGroup(right_front_motor, right_mid_motor, right_back_motor)
drivetrain = SmartDrive(left_motor_group, right_motor_group, inertial_sensor, 260, 320, 320, DistanceUnits.MM, 24/60)

# Monitor Monitoring
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

class Tracking:
    global inertial_sensor, left_motor_group, right_motor_group

    # Tracking wheel geometry
    # In this case we are just uising morot encoders and gyro, however same concept works for odometry wheels
    GEAR_RATIO = 24.0 / 60.0 # external gear ratio
    WHEEL_SIZE = 0.260 # m
    # FWD_OFFSET is the distance from the robot center to the forward tracking wheel, right is positive
    FWD_OFFSET = 0.0 # m
    # SIDE_OFFSET is the distance from the robot center to the side tracking wheel, forward is positive
    SIDE_OFFSET = 0.0 # m

    def __init__(self, x, y, heading):
        self.x = x # meters NORTH
        self.y = y # meters EAST
        # heading is assumed to be a raw inertial sensor reading for initialization
        # heading passed in as degrees 0 to 360. Converted to continuous radians
        # theta is our internal heading in radians
        self.theta = self.to_angle(radians(heading)) / GYRO_SCALE # continuous radians
        self.timestep = 0.01 # seconds

        self.previous_left_position = 0.0 # revolutions
        self.previous_right_position = 0.0 # revolutions
        self.previous_theta = 0.0 # radians

    # helper function for converting angles
    # mimics inertial.angle() producing result in range (-180, 180])
    # note: degrees in and out
    def to_angle(self, angle):
        angle = angle % 360.0
        if (angle > 180.0):
            angle -= 360.0
        return angle

    # mimics inertial.heading() producing result in range [0, 360)
    # note: degrees in and out
    def to_heading(self, angle):
        angle = angle % 360.0
        return angle
    
    # returns internal theta (radians) to degrees heading [0, 360)
    def current_heading(self):
        heading_deg = degrees(self.theta)
        return self.to_heading(heading_deg)

    def calc_timestep_arc_chord(self, x, y, theta, delta_forward, delta_side, delta_theta):
        # x, y, delta_forward, delta_side in meters
        # theta, delta_theta in radians

        # local deltas
        if (delta_theta == 0.0):
            # no turn - use simple deltas
            delta_local_x = delta_forward
            delta_local_y = delta_side
            to_global_rotation_angle = theta
        else:
            # robot turning
            # calculate radius of movement for forward and side wheels
            r_linear = Tracking.FWD_OFFSET + (delta_forward / delta_theta) # m
            r_strafe = Tracking.SIDE_OFFSET + (delta_side / delta_theta) # m

            # calculate chord distances using chord length = 2 * r * sin(theta / 2)
            # pre-rotate by half the turn angle so we have only distance along one axis for each
            # when we rotate to global frame we need to account for this half-angle rotation
            to_global_rotation_angle = theta + delta_theta / 2
            delta_local_x = r_linear * 2.0 * sin(delta_theta / 2.0)
            delta_local_y = r_strafe * 2.0 * sin(delta_theta / 2.0)

        # rotate to global
        delta_global_x = delta_local_x * cos(to_global_rotation_angle) - delta_local_y * sin(to_global_rotation_angle)
        delta_global_y = delta_local_x * sin(to_global_rotation_angle) + delta_local_y * cos(to_global_rotation_angle)

        return (x + delta_global_x, y + delta_global_y, theta + delta_theta)

    def update_location(self, left_position, right_position, theta):
        left_position *= Tracking.GEAR_RATIO
        right_position *= Tracking.GEAR_RATIO

        delta_left = left_position - self.previous_left_position
        delta_right = right_position - self.previous_right_position
        delta_theta = theta - self.previous_theta

        delta_forward = Tracking.WHEEL_SIZE * (delta_left + delta_right) / 2.0
        delta_side = 0.0 # no side encoder

        self.x, self.y, self.theta = self.calc_timestep_arc_chord(self.x, self.y, self.theta, delta_forward, delta_side, delta_theta)

        self.previous_left_position = left_position
        self.previous_right_position = right_position
        self.previous_theta = theta

    def gyro_rotation(self, sensor):
        return radians(sensor.rotation() / GYRO_SCALE)

    @staticmethod
    def tracker_thread():
        # print(args)
        tracker = Tracking(0, 0, inertial_sensor.heading())
        loop_count = 0
        while(True):
            tracker.update_location(left_motor_group.position(RotationUnits.REV), right_motor_group.position(RotationUnits.REV), tracker.gyro_rotation(inertial_sensor))
            #if (loop_count % 100 == 0):
            #    print("X: {:.2f} m, Y: {:.2f} m, Heading: {:.2f} deg".format(tracker.x, tracker.y, tracker.heading()))
            loop_count += 1

def pre_autonomous():
    global initialization_complete
    # actions to do when the program starts
    # wait a bit before doing anything to let devices initialize
    print("pre-auton code")

    wait(0.1, SECONDS)
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")

    color1.set_light_power(100, PERCENT)
    color2.set_light_power(100, PERCENT)

    # calibrate the inertial sensor
    if (inertial_sensor.installed):
        inertial_sensor.calibrate()
        while inertial_sensor.is_calibrating():
            wait(10, TimeUnits.MSEC)

    # start background threads
    motor_monitor_thread = Thread(MotorMonitor.motor_monitor_thread)
    tracker_thread = Thread(Tracking.tracker_thread)
    wait(0.1, SECONDS)

    print("a piston: ", arm_solenoid.value())
    print("b piston: ", ramp_solenoid.value())

    initialization_complete = True

def autonomous():
    # wait for initialization code
    while (not initialization_complete or tests_running):
        wait(10, MSEC)

    print("autonomous code")
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    brain.screen.new_line()
    if (inertial_sensor.installed):
        brain.screen.print("GYRO OK")
    else:
        brain.screen.print("GYRO MISSING")

    left_motor_group.stop(COAST)
    right_motor_group.stop(COAST)

class MedianFilter3:
    def __init__(self, initial_value = 0.0):
        self.buffer = [initial_value] * 3
        self.a = self.b = self.c = initial_value
        self.latest_value = initial_value

    def _update(self, new_value):
        self.buffer.pop(0)
        self.buffer.append(new_value)
        sorted = self.buffer.copy()
        sorted.sort()
        return sorted[1]

    def _update_fast(self, new_value):
        self.a = self.b
        self.b = self.c
        self.c = new_value

        if self.a > self.b:
            if self.b > self.c: return self.b
            elif self.a > self.c: return self.c
            else: return self.a
        else:
            if self.a > self.c: return self.a
            elif self.b > self.c: return self.c
            else: return self.b

    def update(self, new_value):
        self.latest_value = self._update_fast(new_value)
        return self.latest_value
    
    @property
    def median(self):
        return self.latest_value

class DriverControl:
    # Constants for ramp control
    MAX_CONTROL_RAMP = 4 # percent per timestep (assumed to be 10ms)
    
    # Constants to convert percent to volt for drivetrain
    MOTOR_MAXVOLT = 11.5 # volts
    MOTOR_VOLTSCALE = MOTOR_MAXVOLT / 100.0

    # Constant for controller deadband - below this value treat motors as stopped. Avoids robot creep and potential chattering
    MOTOR_DEADBAND = 5

    def __init__(self, left_motor_group, right_motor_group, inertial_sensor, gyro_scale = 1.0):
        self.lmg = left_motor_group
        self.rmg = right_motor_group
        self.gyro = inertial_sensor
        self.gyro_scale = gyro_scale
    
        # Ramp control variables - assumes robot is stationary at start
        self.last_speed = 0
        self.last_turn = 0

        # Motor control variables - 
        #  - drivetrain_running says if drivetrain motors can spin. If false, the controller is in the deadband range and we stop the motors 
        self.drivetrain_running = False

        self.follow_heading = None

    # DETWITCH - reduce turn sensitiviy when robot is moving slowly (turning in place)
    # NOTE: speed is not altered only turn
    # @param speed in percent - from -100 to +100 (full reverse to full forward)
    # @param turn in percent - from -100 to +100 (full left turn to full right turn)
    # returns speed (unmodified) and turn based on algorithm below
    def drivetrain_detwitch(self, speed, turn):   

        speedMixLim = 33.0 # upper limit of throttle mixing (above this point, full turn allowed) in PERCENT
        # NOTE: Next 2 parameters should add up to 100 (throttlemixMinSens + throttlemixSlope = 100)
        speedMixMinSens = 50.0 # PERCENT, minimum turn sensitivity point (i.e. when turning in place)
        speedMixSlope = 50.0 # rate at which turn sensitivity increases with increased throttle
        # turnscale will be used to change how fast we can turn based on spee
        turnscale = 1.0 # start with full turn speed

        if (abs(speed) < speedMixLim):
            speedmix = abs(speed) / speedMixLim
            turnscale = turnscale * ((speedMixMinSens / 100.0) + (speedMixSlope / 100.0) * speedmix)

        turn = turn * turnscale

        return speed, turn

    # RAMP LIMIT - limit how fast we can go from one extreme to another on the joysticks
    # The max range is 200 percent (ie from -100 to +100). A value of 20 for MAX_CONTROL_RAMP would take 0.1s to go from full
    #  forward to full reverse, or from full left to full right turn
    # NOTE: This is done on the control inputs to avoid potential motion artifacts if done on motors separately
    # @param speed is percent forward/reverse speed
    # @param turn is percent left/right speed
    # returns ramp controlled speed and turn (in percent)
    def drivetrain_ramp_limit(self, speed, turn):

        if (abs(speed - self.last_speed) > DriverControl.MAX_CONTROL_RAMP):
            if (speed > self.last_speed): speed = self.last_speed + DriverControl.MAX_CONTROL_RAMP
            else: speed = self.last_speed - DriverControl.MAX_CONTROL_RAMP

        if (abs(turn - self.last_turn) > DriverControl.MAX_CONTROL_RAMP):
            if (turn > self.last_turn): turn = self.last_turn + DriverControl.MAX_CONTROL_RAMP
            else: turn = self.last_turn - DriverControl.MAX_CONTROL_RAMP

        self.last_speed = speed
        self.last_turn = turn

        return speed, turn
    
    # CLAMPING - limits output to range -clamp_value, clamp_value
    # @param input in percent
    # @param (optional) clamp_value - defaults to 100 percent
    # returns clamped value (in percent)
    def clamp(self, input, clamp_value = 100):
        return max(min(input, clamp_value), -clamp_value)
    
    # CONTROLLER_DEADBAND - used in case controller has some drift, mostly for turning
    def controller_deadband(self, input, deadband, max_range = 100):
        output = 0
        scale = max_range / (max_range - deadband)
        if (abs(input) < deadband):
            output = 0
        elif (input > 0):
            output = (input - deadband) * scale
        elif (input < 0):
            output = (input + deadband) * scale

        return output
    
    # GYRO_ROTATION - get the current rotation value in degrees and scale by the gyro scale
    def gyro_rotation(self):
        return self.gyro.rotation() / self.gyro_scale

    # DRIVE_STRAIGHT - will attempt to follow gyro heading when enabled
    def drive_straight(self, speed):
        if (self.gyro is None or not self.gyro.installed):
            return speed, 0
        
        if (self.follow_heading is None):
            self.follow_heading = self.gyro_rotation()
            # print("Follow enabled", self.follow_heading)
            return speed, 0

        error = self.follow_heading - self.gyro_rotation()
        Kp = 3.0
        turn = error * Kp

        # Note: motors turn 10 revolutions per robot 360deg rotation, or 10 * 24 / 60 = 4 wheel rotations
        # - Wheels travel a distance of 4 * 260mm = 1.04m
        # - To turn robot 360deg in one second is 10 revs / sec or 600RPM (coincidently full speed of the blue motors)
        # - Therefore one percent of turn command corresponds to 3.6deg/sec of robot rotational velocity
        # - With Kp of 4 it means we are commanding robot to turn at 14.4deg/sec per degree of heading error

        return speed, turn
    
    # CANCEL_DRIVE_STRAIGHT - resets the heading
    def cancel_drive_straight(self):
        if (self.follow_heading is not None):
            # print("Follow cancelled")
            self.follow_heading = None

    # USER DRIVETRAIN - main entry for user control. Should be called every 10ms
    # First calls the detwitch function
    # Secondly calls the ramp control
    # Thirdly checks deadband range and instructs motors to either run or stop
    # The deadband logic may seem a bit convoluted, but it prevents the motor from being "stopped" every cycle
    # - drivetrain_running is used as a flag so we only stop once until the controls move above the deadband again
    # @param control_speed is raw controller forward / backwards speed in percent
    # @param control_turn is raw controller left / right speed in percent
    # no return value
    def user_drivetrain(self, control_speed, control_turn, enable_drive_straight = False):
        # calculate the drivetrain motor velocities from the controller joystick axes

        # just in case - make sure there is no turn coming from the joystick unless we want it
        control_turn = self.controller_deadband(control_turn, 2)

        # Select auto follow heading mode if enabled and we are not commanded to turn, and are not waiting on a turn to finish
        if (enable_drive_straight and control_speed != 0 and control_turn == 0 and self.last_turn == 0):
            auto_speed, auto_turn = self.drive_straight(control_speed)
            safe_speed, _ = self.drivetrain_ramp_limit(auto_speed, 0)
            safe_turn = auto_turn
        # Else just follow along with what the driver is doing
        else:
            self.cancel_drive_straight()
            detwitch_speed, detwitch_turn = self.drivetrain_detwitch(control_speed, control_turn)
            safe_speed, safe_turn = self.drivetrain_ramp_limit(detwitch_speed, detwitch_turn)
        
        # mix together speed and turn and clamp combined values to -100 to +100 percent
        drivetrain_left_side_speed = self.clamp(safe_speed + safe_turn)
        drivetrain_right_side_speed = self.clamp(safe_speed - safe_turn)

        # check if the values are inside of the deadband range
        if abs(drivetrain_left_side_speed) < DriverControl.MOTOR_DEADBAND and abs(drivetrain_right_side_speed) < DriverControl.MOTOR_DEADBAND:
            # check if the motors have already been stopped
            if self.drivetrain_running:
                # stop the drive motors
                self.lmg.stop()
                self.rmg.stop()
                # tell the code that the motors have been stopped
                self.drivetrain_running = False
        else:
        # reset the toggle so that the deadband code knows to stop the motors next
        # time the input is in the deadband range
            self.drivetrain_running = True

        # NOTE: supplying VOLT shows as a mismatched type error, although it runs fine as is valid per the API. 'type: ignore' silences the error
        # only tell the left drive motor to spin if the values are not in the deadband range
        if self.drivetrain_running:
            self.lmg.spin(FORWARD, drivetrain_left_side_speed * DriverControl.MOTOR_VOLTSCALE, VOLT) # type: ignore

        # only tell the right drive motor to spin if the values are not in the deadband range
        if self.drivetrain_running:
            self.rmg.spin(FORWARD, drivetrain_right_side_speed * DriverControl.MOTOR_VOLTSCALE, VOLT) # type: ignore

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

def open_trapdoor():
    ramp_solenoid.set(1)

def close_trapdoor():
    ramp_solenoid.set(0)

def trapdoor_is_open():
    return ramp_solenoid.value() == 1

# Controller mapping - note there are two control modes:
# - individual control uses all four bumpers L1, L2, R1, R2 and controls intake and shooter independently
# - linked control mapped as follors:
#   - R1: intake
#   - R2: eject
#   - L1: long goal score (intake + shooter)
#   - L2: mid-level goal score (intake + trapdoor open)
# All buttons are toggles in both modes
control_mode = 1 # 0 is individual control, 1 is linked control

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

# Arm solenoid toggle
def OnButtonAPressed():
    if (arm_solenoid.value() == 0):
        arm_solenoid.set(1)
    else:
        arm_solenoid.set(0)

# Wedge solenoid toggle
def OnButtonBPressed():
    if (ramp_solenoid.value() == 0):
        ramp_solenoid.set(1)
    else:
        ramp_solenoid.set(0)

def detect_blue(sensor):
    if not sensor.is_near_object():
        return False
    hue = sensor.hue()
    if hue > 200 and hue < 220:
        return True
    return False

# Expermimental color sort function - only scores red right now (rejects blue)
color_sort_enable = False
color_sort_valid = False
color_sort_valid_count = 0
color_sort_shooter_speed = 0

def color_sort():
    global color_sort_valid, color_sort_valid_count, color_sort_shooter_speed
    if detect_blue(color1) or detect_blue(color2):
        if not color_sort_valid:
            ramp_solenoid.set(1)
            color_sort_shooter_speed = shooter_speed
            shooter_motor.stop(COAST)
            print("Blue detected")
        color_sort_valid = True
        color_sort_valid_count = 0

    elif color_sort_valid:
        if color_sort_valid_count < 40:
            color_sort_valid_count += 1
        else:
            ramp_solenoid.set(0)
            shooter_motor.spin(FORWARD, color_sort_shooter_speed, PERCENT)
            color_sort_valid = False
            print("Blue lost")

def OnButtonXPressed():
    global color_sort_enable
    if color_sort_enable:
        color_sort_enable = False
    else:
        color_sort_enable = True

def user_control():
    # wait for initialization code
    while (not initialization_complete or tests_running):
        wait(10, MSEC)

    print("driver control")
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    # events
    controller_1.buttonR1.pressed(OnButtonR1Pressed) # Intake
    controller_1.buttonR2.pressed(OnButtonR2Pressed) # Eject
    controller_1.buttonL1.pressed(OnButtonL1Pressed) # Shooter On or Long Goal Score
    controller_1.buttonL2.pressed(OnButtonL2Pressed) # Shooter Reverse or Mid Level Goal Score
    controller_1.buttonUp.pressed(OnButtonUpPressed) # Enable / disable drive straight - DEMO ONLY
    controller_1.buttonA.pressed(OnButtonAPressed) # Toggle arm solenoid
    controller_1.buttonB.pressed(OnButtonBPressed) # Toggle trapdoor solenoid
    controller_1.buttonX.pressed(OnButtonXPressed) # Toggle color sort

    # drivetrain control
    driver_control = DriverControl(left_motor_group, right_motor_group, inertial_sensor, GYRO_SCALE)

    # place driver control in this while loop
    while True:
        driver_control.user_drivetrain(controller_1.axis3.position(), controller_1.axis1.position(), ENABLE_DRIVE_STRAIGHT)
        if color_sort_enable and shooter_speed != 0:
            color_sort()
        wait(10, MSEC) # DO NOT CHANGE

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
pre_autonomous()

