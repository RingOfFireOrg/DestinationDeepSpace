package frc.robot;

public class RobotMap {
	// Ports not on robot

	// Joysticks
	public static final int DRIVER_GAMEPAD = 0;
	public static final int MANIPULATOR_GAMEPAD = 1;
	public static final int MANIPULATOR_PANEL = 2; 
	public static final int LEFT_DRIVE_JOYSTICK = 3;
	public static final int RIGHT_DRIVE_JOYSTICK = 4;

	// Manipulator panel buttons
	public static final int AUTO_INTAKE_HATCH_BUTTON = 1;
	public static final int AUTO_SCORE_HATCH_BUTTON = 2;
	public static final int OPEN_BEAK_BUTTON = 3;
	public static final int CLOSE_BEAK_BUTTON = 4;
	public static final int AUTO_MID_ROCKET_CARGO_SCORE_BUTTON = 5;
	public static final int AUTO_LOW_ROCKET_CARGO_SCORE_BUTTON = 6;
	public static final int AUTO_CARGOSHIP_CARGO_SCORE_BUTTON = 7;
	public static final int CARGO_ARM_UP_POSITION_BUTTON = 8;
	public static final int CARGO_ARM_MID_ROCKET_POSITION_BUTTON = 9;
	public static final int CARGO_ARM_LOW_ROCKET_POSITION_BUTTON = 10;
	public static final int CARGO_ARM_CARGO_SHIP_POSITION_BUTTON = 11;
	public static final int CARGO_ARM_INTAKE_POSITION_BUTTON = 12;
	public static final int CLIMBING_MODE_PROTECTED_SWITCH = 16;

	public static final int INTAKE_WHEELS_SPEED_JOYSTICK_AXIS = 1;

	// Manipulator gamepad buttons and axis:
	public static final int MANIPULATOR_LEFT_STICK_Y_AXIS = 1;
	public static final int MANIPULATOR_LEFT_TRIGGER_AXIS = 2;
	public static final int MANIPULATOR_RIGHT_TRIGGER_AXIS = 3;

	public static final int MANIPULATOR_A_BUTTON_VALUE = 1;
	public static final int MANIPULATOR_B_BUTTON_VALUE = 2;
	public static final int MANIPULATOR_X_BUTTON_VALUE = 3;
	public static final int MANIPULATOR_Y_BUTTON_VALUE = 4;
	public static final int MANIPULATOR_LEFT_BUMPER_BUTTON_VALUE = 5;
	public static final int MANIPULATOR_RIGHT_BUMPER_BUTTON_VALUE = 6;
	public static final int MANIPULATOR_BACK_BUTTON_VALUE = 7;
	public static final int MANIPULATOR_START_BUTTON_VALUE = 8;

	// Driver Gamepad Buttons axes:
	public static final int LEFT_STICK_X_AXIS = 0;
	public static final int LEFT_STICK_Y_AXIS = 1;
	public static final int LEFT_TRIGGER_AXIS = 2;
	public static final int RIGHT_TRIGGER_AXIS = 3;
    public static final int RIGHT_STICK_X_AXIS = 4;
	public static final int RIGHT_STICK_Y_AXIS = 5;


	public static final int TEST_FRONT_LEFT_BUTTON = 0;
	public static final int TEST_FRONT_RIGHT_BUTTON = 1;
	public static final int TEST_BACK_LEFT_BUTTON = 2;
	public static final int TEST_BACK_RIGHT_BUTTON = 3;
	public static final int BACK_BUTTON_VALUE = 7;
	public static final int START_BUTTON_VALUE = 8;

	//Left Drive Stick Buttons:
	public static final int JOYSTICK_ROBOT_FRONT_SET_CARGO = 6;
	public static final int JOYSTICK_ROBOT_FRONT_SET_HATCH = 4;

	//Right Drive Stick Buttons:
	public static final int CLIMBER_ENABLE_BUTTON = 7;
	public static final int JOYSTICK_RESET_GYRO_BUTTON = 12;

	// Analog Ports
	// Robot Ports
	public static final int ENCODER_FRONT_RIGHT = 0;
	public static final int ENCODER_FRONT_LEFT = 1;
	public static final int ENCODER_BACK_LEFT = 2;
	public static final int ENCODER_BACK_RIGHT = 3;
	public static final int RIGHT_ENCODER_CARGO_ARM = 4;
	public static final int LEFT_ENCODER_CARGO_ARM = 5;

	// PWM ports
	// Motors:
	public static final int CAMERA_ROTATION_SERVO_CHANNEL = 0;
	public static final int CAN_CLIMBER_BACK = 1;
	public static final int BEAK_ACTUATOR_CHANNEL = 2;

	// Digital Ports
	// Hall Effect Sensors and Limit Switches for autoClimb
	public static final int INPUT_FRONT_SW = 8;
	public static final int INPUT_BACK_SW = 9;

	public static final int INPUT_FRONT_LEFT_WHEEL = 3;
	public static final int INPUT_BACK_LEFT_WHEEL = 0;
	public static final int INPUT_FRONT_RIGHT_WHEEL = 2;
	public static final int INPUT_BACK_RIGHT_WHEEL = 1;

	// need to get real numbers for drive encoders
	public static final int DRIVE_ENCODER_FRONT_RIGHT_A = 10;
	public static final int DRIVE_ENCODER_FRONT_RIGHT_B = 11;
	public static final int DRIVE_ENCODER_FRONT_LEFT_A = 12;
	public static final int DRIVE_ENCODER_FRONT_LEFT_B = 13;
	public static final int DRIVE_ENCODER_BACK_LEFT_A = 18;
	public static final int DRIVE_ENCODER_BACK_LEFT_B = 19;
	public static final int DRIVE_ENCODER_BACK_RIGHT_A = 16;
	public static final int DRIVE_ENCODER_BACK_RIGHT_B = 17;

	// all CANs for robot
	public static final int CAN_CLIMBER_FRONT = 10;
	// can climber wheels are coded as pwm -- check where it is wrong
	public static final int CAN_CLIMBER_WHEELS = 3;

	public static final int LEFT_INTAKE_WHEEL = 8;
	public static final int RIGHT_INTAKE_WHEEL = 7;
	public static final int CARGO_ARM = 6;

	public static final int DRIVE_FRONT_RIGHT_MOTOR = 11;
	public static final int DRIVE_FRONT_LEFT_MOTOR = 14;
	public static final int DRIVE_BACK_LEFT_MOTOR = 5;
	public static final int DRIVE_BACK_RIGHT_MOTOR = 1;

	public static final int STEER_FRONT_RIGHT_MOTOR = 12;
	public static final int STEER_FRONT_LEFT_MOTOR = 13;
	public static final int STEER_BACK_LEFT_MOTOR = 4;
	public static final int STEER_BACK_RIGHT_MOTOR = 2;



	// encoder zero values
	public static final int ENCODER_ZERO_VALUE_FRONT_RIGHT = 209;
	public static final int ENCODER_ZERO_VALUE_FRONT_LEFT = 320;
	public static final int ENCODER_ZERO_VALUE_BACK_LEFT = 43;
	public static final int ENCODER_ZERO_VALUE_BACK_RIGHT = 18;

	// Motor Speeds
	public static final double SPEED_DEFAULT_TEST = 0.75;

	public static final double SPEED_DEFAULT_DRIVE = 0.75;
	public static final double SPEED_DEFAULT_CLIMB = 1.0;
	public static final double BACK_CLIMBER_SPEED_MULTIPLE = 0.95;
	public static final double SPEED_STOP = 0;
	public static final double SPEED_SLOW_CLIMB = 0.6;

	public static final double DEFAULT_FIND_SPEED = 0.5;

	// Wheels
	public static final double WHEEL_DIAMETER = 4;
	public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

	// dimensions of the robot in CM
	public static final int ROBOT_X_IN_CM = 51;
	public static final int ROBOT_Y_IN_CM = 51;

	//Constants
	public static final int SWERVE_WHEEL_DIAMETER = 6;
	public static final double inchToCM = 2.54;
	public static final double pi = 3.14;

	// deadzones
	public static final double TRANSLATION_DEADZONE = 0.1;
	public static final double ROTATION_DEADZONE = 0.1;
	public static final double ABSOLUTE_ROTATION_DEADZONE = 0.3;
	public static final double CARGO_ARM_ROTATION_SPEED = 0.75;

	// wheels speeds on cargo manipulator
	public static final double RIGHT_CARGO_WHEEL_OFF_SPEED = 0;
	public static final double LEFT_CARGO_WHEEL_OFF_SPEED = 0;
	public static final double RIGHT_CARGO_WHEEL_SHOOT_SPEED = 1;
	public static final double LEFT_CARGO_WHEEL_SHOOT_SPEED = -1;
	public static final double RIGHT_CARGO_WHEEL_INTAKE_SPEED = -0.6;
	public static final double LEFT_CARGO_WHEEL_INTAKE_SPEED = 0.6;

	// arm speed on cargo manipulator
	public static final double ARM_RAW_SPEED = 0.4;

	//null value return in cargo manipulator for dual encoder failure
	public static final double FAILURE_RETURN_ENCODER_VALUE = -6;
}
