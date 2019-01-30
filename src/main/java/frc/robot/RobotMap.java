package frc.robot;

public final class RobotMap {

    // All CANs for Robot
    // Example: public static final int CAN_CLIMBER = 13;
    public static final int CAN_TEST_ATTACHMENT = 9;
    public static final int CAN_LIFTER_1 = 7;
	public static final int CAN_LIFTER_2 = 8;

    // All PWMs for Robot
    // Example: public static final int PWM_DRIVETRAIN_FRONT_LEFT = 2;
    public static final int PWM_TEST_ATTACHMENT = 2;

    // Drive Train Motors
    public static final int MOTOR_LEFT = 1;
    public static final int MOTOR_RIGHT = 0;
    //public static final int MOTOR_BACK_LEFT = 3;
    //public static final int MOTOR_BACK_RIGHT = 1;

    // Digital I/O
    // Example: public static final int IO_UPPER_LIMIT_SW = 6;

    // Drive Train Encoders
    public static final int DRIVE_TRAIN_LEFT_ENCODER_A = 2;
    public static final int DRIVE_TRAIN_LEFT_ENCODER_B = 3;
    public static final int DRIVE_TRAIN_RIGHT_ENCODER_A = 4;
    public static final int DRIVE_TRAIN_RIGHT_ENCODER_B = 5;
    
    public static final int INPUT_UPPER_LIMIT_SW = 6;
	public static final int INPUT_LOWER_LIMIT_SW = 7;
	public static final int LIFT_ENCODER_A = 0;
	public static final int LIFT_ENCODER_B = 1;

    // Motor Speeds
    // Example: public static final double SPEED_DEFAULT_LIFT = 0.75;
    public static final double SPEED_DEFAULT_TEST = 0.75;

    public static final double DEFAULT_LIFT_SPEED = 0.5;
	public static final double MIN_LIFT_SPEED = 0.5;
	public static final double MAX_LIFT_SPEED = 1.0;
	
	public static final double DEFAULT_FIND_SPEED = 0.5;

    // Joystick
    // Example: public static final int JOYSTICK_DRIVE_LEFT = 0;
    public static final int JOYSTICK_DRIVE_LEFT = 0;
    public static final int JOYSTICK_DRIVE_RIGHT = 1;
    public static final int JOYSTICK_MANIPULATOR = 2;

    // Joystick Buttons
    // Example: public static final int BUTTON_LIFT_UP = 6;
    public static final int LIFT_UP_BUTTON = 6;
	public static final int LIFT_DOWN_BUTTON = 7;


}