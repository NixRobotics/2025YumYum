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
        '''
        ### Configuration initializers

        :param configuration.fwd_wheel_size: Circumference of the forward odometry wheel in mm. Set to 0.0 if not present. If only one forward wheel is present,\
            use left regardless if it is mounted on the left of the right of the robot. Motor tracking applies setting to both drivetrain sides
        :param configuration.side_sheel_size: Circumference of the sideways odometry wheel in mm. Set to 0.0 if not present
        :param configuration.fwd_gear_ratio: Is any gear ratio used between the wheel and the rotation sensor or motor
        :param configuration.side_gear_ratio: Is any gear ratio used between the wheel and the rotation sensor
        :param configuration.fwd_offset: Is the offset of the forward tracking wheel relative to the turning center of the robot. Positive is to RIGHT of robot, so\
            wheels mounted on left of robot would be negative
        :param configuration.side_offset: Is the offset of the sideways tracking wheel relative to the turning center of the robot. Positive side is to FRONT of robot,\
            so wheels mounted towards the back of the robot would be negative
        :param configuration.fwd_is_odom: If False left and right wheels are treated as the motor left and right MotorGroup encoders\
            It is assumed that encoders are configured correctly such that FORWARD motion is positive for both left and right encoders and RIGHT motiion\
            is positive to side/strafe encoder
        '''
        pass
   
    def enable(self, enabled = True):
        '''
        ### Docstring for enable
        
        :param self: Description
        :param enabled: Description
        '''
        pass

    def enable_resampling(self, enabled = True):
        '''
        ### Docstring for enable_resampling
        
        :param self: Description
        :param enabled: Description
        '''
        pass

    def current_heading(self):
        '''
        ### Gets current tracker heading in degrees (may differ from inertial sensor due to time lag)

        Theoretically this is same as calling GyroHelper.gyro_heading(), but it will vary due to sampling time of the sensor and accumulator effects

        :returns: Internal theta (radians) converted to degrees heading [0, 360)

        '''
        return 0.0
        
    def get_orientation(self):
        '''
        ### Docstring for get_orientation
        
        :returns: Tracking.Orientation Tuple
        '''
        return Tracking.Orientation(0.0, 0.0, 0.0)

    def set_orientation(self, orientation: Orientation):
        '''
        ### Docstring for set_orientation
        
        :param self: Description
        :param orientation: Description
        :type orientation: Orientation
        '''
        pass

    def trajectory_to_point(self, x, y, reverse = False):
        '''
        ### Returns the distance (in MM) and relative heading (in DEGREES) to the specified coordinate.

        By default the robot is assumed to be driving forward to the target point. Set the "reverse" argument to\
        True if the robot needs to reverse to the target.
        
        :param x: X or NORTH-SOUTH axis coordinate in MM
        :param y: Y or EAST-WEST axis coordinate in MM
        :param (optional) reverse: Indicates that the robot will drive in reverse

        :returns distance, heading: Tuple of distance in MM and relative heading in DEGREES to target.\
        If "reverse" is True then distance will be negated and the heading will reflect the direction the\
        front of the robot needs to be pointing meaning heading-180
        '''
        return 0.0, 0.0

    @staticmethod
    def gyro_theta(sensor: InertialWrapper):
        '''
        ### Docstring for gyro_theta
        
        :param sensor: Description
        :type sensor: InertialWrapper
        '''
        return 0.0

    @staticmethod
    def gyro_rotation(sensor: InertialWrapper):
        '''
        ### Docstring for gyro_rotation
        
        :param sensor: Description
        :type sensor: InertialWrapper
        '''
        return 0.0

class SmartDriveWrapper(SmartDrive):
    '''
    ### SmartDriveWrapper class - use this to create a smart(er) drivetrain

    This class inherits the VEX SmartDrive class that provides basic inertial sensor functionality for turning.

    #### Class Hierarchy:
        - Drivetrain (vex)
            - SmartDrive (vex)
                - SmartDriveWrapper (this class)

    This version extends the SmartDrive functionality by impplementing heading hold when driving straight through
    the drive_straight_for() function.
    
    It also implements full control over PID supporting Kp, Ki and Kd for turning, driving
    and heading hold. Three new functions are added to support this:\\
        - set_turn_constants()\\
        - set_drive_constantss()\\
        - set_heading_lock_constants()\\

    It is also based on the InertialWrapper class that provides scaling for gyros with a (constant) heading error.

    The GPS sensor is currently not suppoorted.

    :param lm: Left motorgroup
    :type lm: MotorGroup
    :param rm: Right motorgroup
    :type rm: MotorGroup
    :param g: Inertial sensor
    :type g: InertialWrapper
    :param (optional) wheelTravel:  The circumference of the driven wheels, default is 320 mm (~4" * pi)
    :param (optional) trackWidth: The trackwidth of the drivetrain, default is 254 mm (10")
    :param (optional) wheelBase:  The wheelBase of the drivetrain, default is 254 mm (10)
    :param (optional) units: The units that wheelTravel, trackWidth and wheelBase are specified in, only MM is supported
    :param (optional) externalGearRatio: Gear ratio used to compensate drive distances if gearing is used
    :returns out: A new SmartDriveWrapper object
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
        If this is too large then turns will not be accurate, if too small then turns may not complete.

        :param value: The new turn threshold in degrees, the default for SmartDriveWrapper is +/-1 degree
        '''
        pass

    def set_turn_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the turning constants for the robot

        SmartdriWewrapper uses a PID controller for turning. This requires three gain constants, generally known
        as Kp Ki and Kd. They set the gain used in the equation that turns angular error into motor velocity.

        Each robot will have a unique set of values that is mostly affected by the surface type, nmotor/gear/wheel
        configuration, robot size and weight.

        Even then these are strictly not constants as different speeds may require a different set of values.

        The settle error, or threshold, is set separately using the set_turn_threshold() method

        :param Kp: The new turn P constant
        :param (optional) Ki: The new turn I constant, default is 0.0
        :param (optional) Kd: The new turn D constant, default is 0.0
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
        conditions. However backlash (gear slop) will add 50x-100x more uncertainty than this making a practical limit for the
        threshold at around +/-5mm.

        #### Examples for maximum resolution:
            - 4" wheels, 18:1 internal gearing, no external gearing: +/- 0.089mm (= +/-0.1deg at motor)
            - 3.25" wheels, 6:1 internal gearing, 24:60 external gearing: +/- 0.087mm (= +/-0.3deg)

         :param value: The new drive threshold in MM, the default is +/-5mm
        '''
        pass

    def set_drive_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the driving constants for the robot

        SmartdriWewrapper uses a PID controller when driving..\\
        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns distance error into motor velocity.

        :param Kp: The new turn P constant
        :param (optional) Ki: The new turn I constant, default is 0.0
        :param (optional) Kd: The new turn D constant, default is 0.0
        '''
        pass

    def set_headling_lock_constants(self, Kp, Ki=0.0, Kd=0.0):
        '''
        ### Set the constants used for driving straight, or heading lock/hold

        SmartdriWewrapper uses PID controllers for turns and driving.\\
        When we want the robot to drive straight we use both controllers at the same time.\\
        The main difference between individual turns and drives is that the heading lock does not have a
        timeout or a settle error (threshold) associated with it.

        Kp is often only used and is set more aggressively than for just individual turns.

        This call replaces the turn constants only for driving straight (drive constants are unaffected)

        These constants, generally known as Kp Ki and Kd, set the gain used in the equation that\\
        turns angular error into motor velocity.

        :param Kp: The new turn P constant
        :param (optional) Ki: The new turn I constant, default is 0.0
        :param (optional) Kd: The new turn D constant, default is 0.0
        '''
        pass

    def set_heading(self, value, units=RotationUnits.DEG):
        '''
        ### Set the inertial sensor heading to a new value

        The new value for heading should be in the range [0, 360) degrees.

        :param value: The new value to use for heading.
        :param (optional) units: The rotation units type for value, only DEGREES is supported
        '''
        pass
    
    def heading(self, units=RotationUnits.DEG):
        '''
        ### Read the current heading of the inertial sensor

        Heading will be returned in the range [0, 360) degrees

        :param (optional) units: The units to return the heading in, only DEGREES is supported
        :returns heading: in DEGREES
        '''
        return 0.0
    
    def set_rotation(self, value, units=RotationUnits.DEG):
        '''
        ### Set the inertial sensor rotation to a new value

        :param value: The new value to use for rotation.
        :param (optional) units: The rotation units type for value, only DEGREES is supported
        '''
        pass

    def rotation(self, units=RotationUnits.DEG):
        '''
        ### Read the current rotation of the inertial sensor

        rotation is unbounded, meaning it can be in the range [-inf, inf]. This is generally more useful for performing
        calculations on a heading as there are no discontinuities when going from (say) a heading of 359 to 0 degrees.

        heading is just the same value as rotation but "reduced" and offset so that it is in the range of [0, 359). heading
        is more useful for specifying directions for the robot

        :param (optional) units: The rotation units type for value, only DEGREES is supported
        :returns: A value for heading in the range that is specified by the units.
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
        ### turn the robot to an absolute heading

        The angle and direction by which to turn the robot is calculated automatically. It will turn by the smallest
        angle to get to the specified heading. A difference of exactly 180deg will turn to the RIGHT.

        :param heading: The heading to turn to in the range [0, 360)
        :pafram (optional) units: The units for the provided angle (DEGREES only)
        :param (optional) velocity: spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
        :param (optional) units_v: The units of the provided velocity (PERCENT only)
        :param (optional) wait: This indicates if the function should wait for the command to complete or return immediately, default is True.
        :returns time_taken: Returns the time taken in MS (if wait=True), else returns 0
        '''
        return 0.0

    def turn_to_rotation(self, rotation, units=RotationUnits.DEG,
                         velocity=None, units_v:  VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### turn the robot to an absolute rotation

        Care should be taken when using this version as rotation can be outside of the range -360,360 deg if the robot has
        performed one or more complete turns. E.g. if the current rotation is 360deg and 0 deg is specified, the robot will
        turn by 360deg (not zero) to make the rotation match the target

        :param rotation: The rotation to turn to
        :pafram (optional) units: The units for the provided angle (DEGREES only)
        :param (optional) velocity: spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
        :param (optional) units_v: The units of the provided velocity (PERCENT only)
        :param (optional) wait: This indicates if the function should wait for the command to complete or return immediately, default is True.
        :returns time_taken: Returns the time taken in MS (if wait=True), else returns 0
        '''
        return 0.0

    def turn_for(self, direction, angle, units = RotationUnits.DEG,
                 velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT, wait=True):
        '''
        ### Turn the robot for an angle left or right

        The robot is commanded to turn by specified angle either LEFT or RIGHT. Note that turning by negative amount\\
        is the same as providing a positive value but switching direction from LEFT to RIGHT or vice-versa

        :param direction: The direction to turn, LEFT or RIGHT
        :param angle: The angle to turn
        :param (optional) units: The units for the provided angle (DEGREES only)
        :param (optional) velocity: drive using this velocity, the default velocity set by set_drive_velocity will be used if not provided.
        :parm (optional) units_v: The units of the provided velocity (PERCENT only)
        :param (optional) wait: This indicates if the function should wait for the command to complete or return immediately, default is True.
        :returns time_taken: Returns the time taken in MS (if wait=True), else returns 0
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
        '''
        ### Drives the robot for the distance specified

        If velocity is not specified the value provided by set_drive_velocity() will be used.

        Internally the function uses the wheelSize and externalGearRatio specified upon object creation to determine how
        many revolutions the motor needs to turn for, so make sure these are set correctly
        
        :param direction: FOWARD or REVERSE
        :param distance: Distnace in MM the robot should drive. Providing negative distance will drive in the opposite direction specified by the direction parameter
        :param (optional) units: Only MM supported
        :param velocity: Description
        :param units_v: Description
        :type units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits
        :param wait: Description
        :return: Description
        :rtype: Any | Literal[True]
        '''
        return 0.0

    def drive_straight_for(self, direction, distance, units = DistanceUnits.MM,
                  velocity=None, units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits = VelocityUnits.PERCENT,
                  heading = None, units_h = RotationUnits.DEG,
                  wait=True):
        '''
        ### Docstring for drive_straight_for
        
        :param direction: Description
        :param distance: Description
        :param units: Description
        :param velocity: Description
        :param units_v: Description
        :type units_v: VelocityUnits.VelocityUnits | PercentUnits.PercentUnits
        :param heading: Description
        :param units_h: Description
        :param wait: Description
        :return: Description
        :rtype: Any | Literal[True]
        '''
        return 0.0

    def drive(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''
        ### NOT SUPPORTED
        '''
        pass

    def is_turning(self):
        '''
        ### DEPRECATED: Use is_done()
        '''
        return False

    def is_moving(self):
        '''
        ### DEPRECATED: Use is_done() 
        '''
        return False

    def is_done(self):
        '''
        ### Returns the current status of the drive_for or turn_for command

        This function is used when False has been passed as the wait parameter to drive_for or turn_for\\
        It will return False if the drivetrain is still moving or True if it has completed the move or a timeout occurred.

        :returns: The current drive_for or turn_for status
        '''
        return False

    def set_drive_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for drive commands

        This will be the velocity used for subsequent calls to drive if a velocity is not provided
        to that function.

        :param velocity: The new velocity
        :param (optional) units: Only PERCENT is supported
        '''
        pass

    def set_drive_accleration(self, accel, units:VelocityPercentUnits = VelocityUnits.PERCENT):
        '''
        ### Set default acceleration for drive commands

        This will be the accelertaion used for subsequent calls to drive

        :param accel: The new acceleration in PERCENT per timestep
        :param (optional) units: Only PERCENT is supported
        '''
        pass

    def set_turn_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.PERCENT):
        '''
        ### Set default velocity for turn commands
        
        This will be the velocity used for subsequent calls to turn if a velocity is not provided
        to that function.

        :param velocity: The new velocity
        :param (optional) units: The units for the supplied velocity, (PERCENT only))
        '''
        pass

    def set_stopping(self, mode=BrakeType.COAST):
        '''
        ### Set the stopping mode for all motors on the drivetrain

        Setting the action for the motors when stopped.

        :param (optional) mode: The stopping mode, COAST (default), BRAKE or HOLD
        '''
        pass

    def set_timeout(self, timeout, units=TimeUnits.MSEC):
        '''
        ### Set the timeout value used all motors on the drivetrain
        
        The timeout value is used when performing drive_for and turn_for commands.  If timeout is
        reached and the motor has not completed moving, then the function will return False.

        timeout : The new timeout
        units : The units for the provided timeout, the default is MSEC
        '''
        pass

    def get_timeout(self):
        '''
        ### Get the current timeout value used by the drivetrain

        :returns: Timeout value in mS
        '''
        return 0.0

    def stop(self, mode=None):
        '''
        ### Stop the drivetrain
        
        This will override any running drive or turn commands (as long as they are interruptable, wait=True)

        Velocity is set to 0 and the specified mode (if present) is used to stop the motors.
        
        If mode is not specified, then the default action that was set using set_stopping() will be used.

        :param (optional) mode: The stopping mode, COAST, BRAKE or HOLD
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

    :param left_motor_group: MotorGroup for the left side of the drivetrain
    :param right_motor_group: MotorGroup for the right side of the drivetrain
    :param inertial_sensor: InertialWrapper for the gyro sensor
    '''
    def __init__(self, left_motor_group: MotorGroup, right_motor_group: MotorGroup, inertial_sensor: InertialWrapper):
        pass

    def set_mode(self,
                 enable_drive_straight = None,
                 enable_heading_lock = None,
                 enable_percent_drive = None,
                 enable_brake_mode = None,
                 enable_ramp_control = None,
                 enable_detwitch = None,
                 follow_heading = None,
                 follow_heading_Kp = None):
        '''
        ### Docstring for set_mode
        
        :param enable_drive_straight: Description
        :param enable_heading_lock: Description
        :param enable_percent_drive: Description
        :param enable_brake_mode: Description
        :param enable_ramp_control: Description
        :param enable_detwitch: Description
        :param follow_heading: Description
        :param follow_heading_Kp: Description
        '''
        pass

    def set_speed_limits(self, drive_max = None, turn_max = None, ramp_max = None):
        '''
        ### Docstring for set_speed_limits
        
        :param drive_max: Description
        :param turn_max: Description
        :param ramp_max: Description
        '''      
        pass

    def set_detwitch_params(self, pivot_max_turn = None, pivot_min_drive_speed = None, full_turn_drive_speed = None):
        '''
        ### Docstring for set_detwitch_params
        
        :param pivot_max_turn: Description
        :param pivot_min_drive_speed: Description
        :param full_turn_drive_speed: Description
        '''        
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
