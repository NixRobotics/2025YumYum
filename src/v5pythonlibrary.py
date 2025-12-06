from vex import *

class InertialWrapper(Inertial):
    '''
    ### Wraps the Inertial class to add scaling correction

    API matches that of Inertial with a couple of helper functions added.

    Upon creation a scale factor must be provided along with the port number. This is the correction that needs to \\
    be aplied to get the correct reading. E.g. if a robot it turned 360 degrees and the gyro reads 355 degrees, then \\
    the scale factor should be 360 / 355 (or 1.0141). Another way to determine this is to use a rroutine such as \\
    SmartDrive.turn_for() to turn the robot by 360 degrees. The resulting error in heading will be the inverse of \\
    the scale factor, meaning that if the robot turns by 365 degrees when instructed to turn 360 degrees, the scale \\
    factor is 365 / 360 (or 1.0139). In practice we would use several turns, say 10, in both directions as the error \\
    becomes more visible and easy to measure.

    ### Arguments
        port: port number
        gyro_scale: this is the READOUT scale factor needed to produce the correct reading for a given robot turn

    ### Returns
        Instance of InertialWrapper
    '''
    def __init__(self, port, gyro_scale):
        pass

    def rotation(self, units = RotationUnits.DEG):
        '''
        ### Returns corrected rotation
        '''
        return 0.0

    def heading(self, units = RotationUnits.DEG):
        '''
        ### Returns corrected heading
        '''
        return 0.0

    def angle(self, units = RotationUnits.DEG):
        '''
        ### Returns corrected angle
        '''
        return 0.0
    
    def set_rotation(self, value, units=RotationUnits.DEG):
        '''
        ### Sets the Inertial rotation to the value specified while correcting for scale factor
        '''
        pass
    
    def set_heading(self, value, units=RotationUnits.DEG):
        '''
        ### Sets the Inertial heading to the value specified while correcting for scale factor
        '''
        pass

    @staticmethod
    def to_heading(rotation):
        '''
        ### Helper function to reduce a continues rotation value to a heading

        Used when you want to convert the raw rotation reading that can be in the range [-inf, +inf] \\
        to an absolute heading in the range [0, 360) degrees
        '''
        return 0.0

    @staticmethod
    def to_angle(rotation):
        '''
        ### Helper function to reduce a continues rotation value to an angle

        Used when you want to convert the raw rotation reading that can be in the range [-inf, +inf] \\
        to an absolute angle in the range (-180, 180] degrees
        '''
        return 0.0
    
    def calc_angle_to_rotation(self, rotation):
        '''
        ### Helper function to calculate the angle to an absolute rotation

        Used when you want to determine how far the robot needs to turn to point to a given rotation. \\
        Return value is an angle in the range [-180, 180]. This assumes you want to turn the shortest \\
        distance to the rotation (ie not the long way around)

        ### Arguments
            rotation: in degrees

        ### Returns
            Angle to rotation
        '''
        return 0.0

    def calc_angle_to_heading(self, heading):
        '''
        ### Helper function to calculate the angle to an absolute heading

        Used when you want to determine how far the robot needs to turn to point to a given heading. \\
        Return value is an angle in the range [-180, 180]. This assumes you want to turn the shortest \\
        distance to the heading (ie not the long way around)

        ### Arguments
            heading: in degrees

        ### Returns
            Angle to heading
        '''
        return 0.0

    def calc_rotation_at_heading(self, heading):
        '''
        ### Helper function to calculate the inertial sensor rotation value for a given heading

        As the robot turns, the rotation will keep increasing, unlike heading that resets to zero every \\
        360 degrees. This means that to determine the sensor reading for a given heading we will need \\
        to back-calculate the rotation value

        ### Arguments
            heading: in degrees

        ### Returns
            Sensor rotation reading at that heading
        '''
        return 0.0
    
from collections import namedtuple

class Tracking:
    '''
    ### Tracking class to calculate robot position using tracking wheels or motor encoders plus inertial sensor

    Internally everything is based in meters and radians
    To distinguish from VEX uses for various heading readings, we use the following terminology:
    - ROTATION: continuous rotation value in degrees (can be positive or negative, no bounds)
    - HEADING: bounded heading in degrees [0, 360)
    - ANGLE: bounded angle in degrees (-180, +180]
    - THETA: continuous rotation value in radians (can be positive or negative, no bounds). Same as ROTATION but in radians
    THETA is used internally and converted to/from HEADING/ANGLE as needed
    
    Note that  __init__() and update_location() assume that the gyro scale factor has already been applied to the inertial sensor readings
    '''
    
    Orientation = namedtuple('Orientation', ['x', 'y', 'heading'])
    
    # Configuration initializers
    # @param *_wheel_size is the the size of the odometry wheel. Set to 0.0 if not present. If only one forward wheels is present, use left regardless
    #  if it is mounted on the left of the right of the robot
    # @param *_gear_ratio is any gear ratio used
    # @param *_offset is the offset of the tracking wheel relative to the turning center of the robot. Positive left/right is to RIGHT of robot, so
    #  wheels mounted on left of robot would be negative. Positive side is to FRONT of robot, so wheels mounted towards the back of the robot
    #  would be negative
    # @param fwd_is_odom. If False left and rigght wheels are trated as the motor left and right motor group encoders
    # It is assumed that encoders are configured correctly such that FORWARD motion is positive for both left and right encoders and RIGHT motiion
    # is positive to side/strafe encoder

    Configuration = namedtuple('Configuration', [
            'fwd_is_odom',
            'fwd_wheel_size', 'fwd_gear_ratio', "fwd_offset",
            'side_wheel_size', 'side_gear_ratio', 'side_offset'
       ])
    
    # Encoder initializers
    # @param left is initial left encoder position in revolutions (either left motors or left odom wheek)
    # @param right is initial right encoder position in revolutions (either right motors or right odom wheel)
    # @param side is sideways or strafe encoder in revolutions (only valid if sideways odom wheel installed)
    # @param theta is initial gyro theta in radians (currently not used - the inertial sensor will be programmed to the initial heading)
    # This assumes that when declaring devices that robot forward is positive and robot right is positive. Positive rotation is towards the right
    EncoderValues = namedtuple('EncoderValues', ['left', 'right', 'side', 'theta'])

    def __init__(self,
                 devices,
                 orientation: Union[Orientation, None] = None,
                 configuration: Union[Configuration, None] = None,
                 initial_values: Union[EncoderValues, None] = None,
                 initial_timestamps: Union[EncoderValues, None] = None):
        pass
    
    def set_initial_values(self, initial_values: EncoderValues, initial_timestamps: EncoderValues):
        pass

    def set_configuration(self, configuration: Configuration):
        pass
   
    def enable(self, enabled = True):
        pass

    def enable_resampling(self, enabled = True):
        pass

    def current_heading(self):
        '''
        Returns internal theta (radians) in degrees heading [0, 360)

        Theoretically this is same as calling GyroHelper.gyro_heading()
        '''
        return 0.0
        
    def get_orientation(self):
        return Tracking.Orientation(0.0, 0.0, 0.0)

    def set_orientation(self, orientation: Orientation):
        pass

    def trajectory_to_point(self, x, y):
        return 0.0, 0.0

    @staticmethod
    def gyro_theta(sensor: InertialWrapper):
        return 0.0

    @staticmethod
    def gyro_rotation(sensor: InertialWrapper):
        return 0.0

class SmartDriveWrapper(SmartDrive):
    '''
    ### SmartDriveWrapper class - use this to create a smart(er) drivetrain

    The smart drivetrains use an inertial sensor for heading, taking the guesswork
    out of turning the robot. This version extends the base SmartDrive functionality
    by impplementing heading hold when driving straight and full control over PID.

    It is also based on the InertialWrapper class that provides scaling for gyros with
    a (constant) heading error.

    #### Arguments:
    - lm : Left motorgroup
    - rm : Right motorgroup
    - g : Inertial sensor
    - wheelTravel (optional) : The circumference of the driven wheels, default is 320 mm (~4" * pi)
    - trackWidth (optional) : The trackwidth of the drivetrain, default is 254 mm (10")
    - wheelBase (optional) : The wheelBase of the drivetrain, default is 254 mm (10)
    - units (optional) : The units that wheelTravel, trackWidth and wheelBase are specified in, only MM is supported
    - externalGearRatio (optional) : An optional gear ratio used to compensate drive distances if gearing is used

    #### Returns:
        A new SmartDriveWrapper object.
    '''
    def __init__(self,
                 lm: MotorGroup,
                 rm: MotorGroup,
                 g: InertialWrapper,
                 wheelTravel = 320.0,
                 trackWidth = 254.0,
                 wheelBase = 254.0,
                 units = DistanceUnits.MM,
                 externalGearRatio = 1.0):
        
        pass

    def set_turn_threshold(self, value):
        '''
        ### Set the turning threshold for the robot

        This is the threshold value used to determine that turns are complete.\\
        If this is too large then turns will not be accurate, if too small then turns ma\\
        not complete.

        #### Arguments:
            value : The new turn threshold in degrees, the default for SmartDriveWrapper is +/-1 degree

        #### Returns:
            None
        '''
        pass

    def set_turn_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the turning constants for the robot

        SmartdriWewrapper uses a PID controller when doing turns.\\
        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns angular error into motor velocity.

        The settle error or threshold is set separately using the set_turn_threshold() method

        #### Arguments:
            Kp : The new turn P constant
            Ki (optional) : The new turn I constant, default is 0.0
            Kd (optional) : The new turn D constant, default is 0.0

        #### Returns:
            None
        '''

        pass

    def set_turn_constant(self, Kp):
        '''
        ### DEPRECATED: Use set_turn_constants()
        '''
        pass

    def set_drive_threshold(self, value):
        '''
        ### Set the drive threshold for the robot

        This is the distance threshold value used to determine that drives are complete.\\
        If this is too large then drive distance will not be accurate, if too small then drives may not complete.

        Note that internally within the motor the best accuracy is limited by the encoder which has a resolution
        of 3.6 degrees, or 0.2deg (+/-0.1deg) for a green catridge and 0.6deg (+/-0.3deg) for a blue cartdige.\\
        The impact of this will depend on the wheel size and the external gearing. Some examples are given below for ideal
        conditions. However backlash (gear slop) will add much more uncertainty than this making a practical limit for the
        threshold at around +/-5mm.

        #### Arguments:
        - value : The new drive threshold in MM, the default is +/-5mm

        #### Returns:
            None

        #### Examples for maximum resolution:
            - 4" wheels, 18:1 internal gearing, no external gearing: +/- 0.089mm (= +/-0.1deg at motor)
            - 3.25" wheels, 6:1 internal gearing, 24:60 external gearing: +/- 0.087mm (= +/-0.3deg)
        '''
        pass

    def set_drive_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the turning constants for the robot

        SmartdriWewrapper uses a PID controller when driving.\\
        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns distance error into motor velocity.

        #### Arguments:
            Kp : The new turn P constant
            Ki (optional) : The new turn I constant, default is 0.0
            Kd (optional) : The new turn D constant, default is 0.0

        #### Returns:
            None
        '''

        pass

    def set_headling_lock_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the constants used for driving straight, or heading lock/hold

        SmartdriWewrapper uses PID controllers for turns and driving.\\
        When we want the robot to drive straight we use both controllers at the same time.\\
        The main difference between
        individual turns and drives is that the heading lock does not have a timeout or a settle error (threshold)
        associated with it.
        Kp is often only used and is set more aggressively than for just individual turns.

        This call replaces the turn constants only for driving straight (drive constants are unaffected)

        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns angular error into motor velocity.

        #### Arguments:
        - Kp : The new turn P constant
        - Ki (optional) : The new turn I constant, default is 0.0
        - Kd (optional) : The new turn D constant, default is 0.0

        #### Returns:
            None
        '''

        pass

    def set_heading(self, value, units=RotationUnits.DEG):
        '''
        ### set the inertial sensor heading to a new value

        The new value for heading should be in the range [0, 360) degrees.

        #### Arguments:
            value : The new value to use for heading.
            units (optional) : The rotation units type for value, only DEGREES is supported

        #### Returns:
            None

        #### Examples:
            # set the value of heading to 180 degrees\\
            smart1.set_heading(180)
        '''
        pass
    
    def heading(self, units=RotationUnits.DEG):
        '''
        ### read the current heading of the inertial sensor

        heading will be returned in the range [0, 360) degrees

        #### Arguments:
            units (optional) : The units to return the heading in, only DEGREES is supported

        #### Returns:
            heading in DEGREES

        #### Examples:
            # get the current heading for the robot\\
            value = smart1.heading()
        '''
        return 0.0
    
    def set_rotation(self, value, units=RotationUnits.DEG):
        '''
        ### set the inertial sensor rotation to a new value

        #### Arguments:
            value : The new value to use for rotation.
            units (optional) : The rotation units type for value, only DEGREES is supported

        #### Returns:
            None

        #### Examples:
            # set the value of rotation to 180 degrees\\
            smart1.set_rotation(180)
        '''
        pass

    def rotation(self, units=RotationUnits.DEG):
        '''
        ### read the current rotation of the inertial sensor

        rotation is not limited, it can be both positive and negative and shows the absolute angle of the gyro.

        #### Arguments:
            units (optional) : The rotation units type for value, only DEGREES is supported

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current rotation for the robot\\
            value = smart1.rotation()
        '''
        return 0.0
    
    def set_turn_direction_reverse(self, value):
        '''
        ### NOT SUPPORTED
        '''
        pass

    def turn_to_heading(self, heading, units=RotationUnits.DEG,
                        velocity=None, units_v:VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### turn the robot to an absolute position

        This function uses the value of heading() when turning the robot

        #### Arguments:
            heading : The heading to turn to
            units (optional) : The units for the provided angle (DEGREES only)
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocit (PERCENT only)
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None
        '''
        return 0.0

    def turn_to_rotation(self, rotation, units=RotationUnits.DEG,
                         velocity=None, units_v:  VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### turn the robot to an absolute rotation

        This function uses the value of rotation() when turning the robot

        #### Arguments:
            rotation : The angle to turn to
            units (optional) : The units for the provided angle, the default is DEGREES
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
        '''
        return 0.0

    def turn_for(self, direction, angle, units = RotationUnits.DEG,
                 velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### Turn the robot for an angle left or right

        The robot is commanded to turn by specified angle either LEFT or RIGHT. Note that turning by negative amount\\
        is the same as providing a positive value but switching direction from LEFT to RIGHT or vice-versa

        #### Arguments:
        1. direction : The direction to turn, LEFT or RIGHT
        2. angle : The angle to turn
        3. units (optional) : The units for the provided angle (DEGREES only)
        4. velocity (optional) : drive using this velocity, the default velocity set by set_drive_velocity will be used if not provided.
        5. units_v (optional) : The units of the provided velocity (PERCENT only)
        6. wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None or if wait is True then completion success or failure

        #### Examples:
        '''
        return 0.0

    def turn(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''
        ### NOT SUPPORTED
        '''
        pass

    def drive_for(self, direction, distance, units = DistanceUnits.MM,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                  wait=True):
        return 0.0

    def drive_straight_for(self, direction, distance, units = DistanceUnits.MM,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                  heading = None, units_h = RotationUnits.DEG,
                  wait=True):
        return 0.0

    def drive(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''
        ### NOT SUPPORTED
        '''
        pass

    def is_turning(self):
        '''
        ### DEPRECATED: Use is_done()
    
        ### Returns the current status of the turn_to_heading, turn_to_rotation or turn_for command
        This function is used when False has been passed as the wait parameter to turn_to_heading or turn_for\\
        It will return True if the drivetrain is still moving or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current turn_to_heading, turn_to_rotation or turn_for status
        '''
        return False

    def is_moving(self):
        '''
        ### DEPRECATED: Use id_done() 

        ### Returns the current status of the drive_for command
        This function is used when False has been passed as the wait parameter to drive_for\\
        It will return True if the drivetrain is still moving or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current drive_for status
        '''
        return False

    def is_done(self):
        '''
        ### Returns the current status of the drive_for or turn_for command
        This function is used when False has been passed as the wait parameter to drive_for or turn_for\\
        It will return False if the drivetrain is still moving or True if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current drive_for or turn_for status
        '''
        return False

    def set_drive_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for drive commands
        This will be the velocity used for subsequent calls to drive if a velocity is not provided
        to that function.

        #### Arguments:
            velocity : The new velocity
            units : Only PERCENT is supported

        #### Returns:
            None
        '''
        pass

    def set_drive_accleration(self, accel, units:VelocityPercentUnits = VelocityUnits.PERCENT):
        '''
        ### Set default acceleration for drive commands

        This will be the accelertaion used for subsequent calls to drive

        #### Arguments:
            accel : The new acceleration in PERCENT per timestep
            units : Only PERCENT is supported

        #### Returns:
            None
        '''
        pass

    def set_turn_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for turn commands
        This will be the velocity used for subsequent calls to turn if a velocity is not provided
        to that function.

        #### Arguments:
            velocity : The new velocity
            units : The units for the supplied velocity, (PERCENT only))

        #### Returns:
            None
        '''
        pass

    def set_stopping(self, mode=BrakeType.COAST):
        '''
        ### Set the stopping mode for all motors on the drivetrain
        Setting the action for the motors when stopped.

        #### Arguments:
            mode : The stopping mode, COAST, BRAKE or HOLD

        #### Returns:
            None
        '''
        pass

    def set_timeout(self, timeout, units=TimeUnits.MSEC):
        '''
        ### Set the timeout value used all motors on the drivetrain
        The timeout value is used when performing drive_for and turn_for commands.  If timeout is
         reached and the motor has not completed moving, then the function will return False.

        #### Arguments:
            timeout : The new timeout
            units : The units for the provided timeout, the default is MSEC

        #### Returns:
            None
        '''
        pass

    def get_timeout(self):
        '''
        ### Get the current timeout value used by the drivetrain

        #### Arguments:
            None

        #### Returns:
            Timeout value in mS
        '''
        return 0.0

    def on_timeout(self, fn):
        pass

    def stop(self, mode=None):
        '''
        ### Stop the drivetrain, set to 0 velocity and set current stopping_mode
        The motors will be stopped and set to COAST, BRAKE or HOLD

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

class DriverControl:
    '''
    ### DriverControl class
    This class implements driver control for a tank drive robot using a left and right motor group and an inertial sensor.
    It includes features such as:
    - Deadband handling to prevent motor creep
    - Ramp control to limit acceleration/deceleration rates
    - Detwitching to reduce turn sensitivity at low speeds
    - Gyro-based straight driving assistance
    - Selctable brake modes
    '''
    def __init__(self, left_motor_group: MotorGroup, right_motor_group: MotorGroup, inertial_sensor: InertialWrapper):
        pass

    def set_mode(self,
                 enable_drive_straight = None,
                 enable_percent_drive = None,
                 enable_brake_mode = None,
                 enable_ramp_control = None,
                 enable_detwitch = None):
        
        pass

    def set_speed_limits(self, drive_max = None, turn_max = None, ramp_max = None):
        
        pass

    def set_detwitch_params(self, pivot_max_turn = None, pivot_min_drive_speed = None, full_turn_drive_speed = None):
        
        pass

    def user_drivetrain(self, control_speed, control_turn):
        '''
        ### USER DRIVETRAIN - main entry for user control. Should be called every 10ms
        
        First calls the detwitch function\\
        Secondly calls the ramp control\\
        Thirdly checks deadband range and instructs motors to either run or stop
        
        The deadband logic may seem a bit convoluted, but it prevents the motor from being "stopped" every cycle
         - drivetrain_running is used as a flag so we only stop once until the controls move above the deadband again

        :param control_speed: is raw controller forward / backwards speed in percent
        :param control_turn: is raw controller left / right speed in percent

        :returns: No return value
        '''
        pass
