package frc.robot;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveDrive {
	public static final int DRIVE_FRONT_RIGHT_MOTOR = 0;
	public static final int DRIVE_FRONT_LEFT_MOTOR = 2;
	public static final int DRIVE_BACK_LEFT_MOTOR = 4;
	public static final int DRIVE_BACK_RIGHT_MOTOR = 6;

	public static final int STEER_FRONT_RIGHT_MOTOR = 1;
	public static final int STEER_FRONT_LEFT_MOTOR = 3;
	public static final int STEER_BACK_LEFT_MOTOR = 5;
	public static final int STEER_BACK_RIGHT_MOTOR = 7;

	public static final int ENCODER_ZERO_VALUE_FRONT_RIGHT = 46;
	public static final int ENCODER_ZERO_VALUE_FRONT_LEFT = 25;
	public static final int ENCODER_ZERO_VALUE_BACK_LEFT = 26;
	public static final int ENCODER_ZERO_VALUE_BACK_RIGHT = 182;

	public static final int ENCODER_FRONT_RIGHT = 0;
	public static final int ENCODER_FRONT_LEFT = 1;
	public static final int ENCODER_BACK_LEFT = 2;
	public static final int ENCODER_BACK_RIGHT = 3;

	//need to get real numbers for drive encoders
	public static final int DRIVE_ENCODER_FRONT_RIGHT_A = 12;
	public static final int DRIVE_ENCODER_FRONT_RIGHT_B = 12;
	public static final int DRIVE_ENCODER_FRONT_LEFT_A = 12;
	public static final int DRIVE_ENCODER_FRONT_LEFT_B = 12;
	public static final int DRIVE_ENCODER_BACK_LEFT_A = 12;
	public static final int DRIVE_ENCODER_BACK_LEFT_B = 12;
	public static final int DRIVE_ENCODER_BACK_RIGHT_A = 12;
	public static final int DRIVE_ENCODER_BACK_RIGHT_B = 12;

	//dimensions of the robot in CM
	public static final int ROBOT_X_IN_CM = 51;
	public static final int ROBOT_Y_IN_CM = 51;


	SwerveModule frontLeft;
	SwerveModule frontRight;
	SwerveModule backLeft;
	SwerveModule backRight;

	SwerveDrive() {
		frontRight = new SwerveModule(new Jaguar(DRIVE_FRONT_RIGHT_MOTOR), new Talon(STEER_FRONT_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_FRONT_RIGHT), ENCODER_ZERO_VALUE_FRONT_RIGHT);
				//*new Encoder(DRIVE_ENCODER_FRONT_RIGHT_A, DRIVE_ENCODER_FRONT_RIGHT_B, false, Encoder.EncodingType.k2X));
		frontLeft = new SwerveModule(new Jaguar(DRIVE_FRONT_LEFT_MOTOR), new Talon(STEER_FRONT_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_FRONT_LEFT), ENCODER_ZERO_VALUE_FRONT_LEFT);
				//new Encoder(DRIVE_ENCODER_FRONT_LEFT_A, DRIVE_ENCODER_FRONT_LEFT_B, false, Encoder.EncodingType.k2X));
		backLeft = new SwerveModule(new Jaguar(DRIVE_BACK_LEFT_MOTOR), new Talon(STEER_BACK_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_BACK_LEFT), ENCODER_ZERO_VALUE_BACK_LEFT);
				//new Encoder(DRIVE_ENCODER_BACK_LEFT_A, DRIVE_ENCODER_BACK_LEFT_B, false, Encoder.EncodingType.k2X));
		backRight = new SwerveModule(new Jaguar(DRIVE_BACK_RIGHT_MOTOR), new Talon(STEER_BACK_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_BACK_RIGHT), ENCODER_ZERO_VALUE_BACK_RIGHT);
				//new Encoder(DRIVE_ENCODER_BACK_RIGHT_A, DRIVE_ENCODER_BACK_RIGHT_B, false, Encoder.EncodingType.k2X));
	}

	void individualModuleControl(boolean buttonfr, boolean buttonfl, boolean buttonbr, boolean buttonbl) {
		if (buttonfr) {
			frontRight.control(0.6, 0);
		} else {
			frontRight.stop();
		}
		if (buttonfl) {
			frontLeft.control(0.6, 0);
		} else {
			frontLeft.stop();
		}
		if (buttonbr) {
			backRight.control(0.6, 0);
		} else {
			backRight.stop();
		}
		if (buttonbl) {
			backLeft.control(0.6, 0);
		} else {
			backLeft.stop();
		}
		SmartDashboard.putNumber("front right encoder: ", frontRight.getAngle());
		SmartDashboard.putNumber("front left encoder: ", frontLeft.getAngle());
		SmartDashboard.putNumber("back right encoder: ", backRight.getAngle());
		SmartDashboard.putNumber("back left encoder: ", backLeft.getAngle());

		SmartDashboard.putNumber("Corrected angle FR", frontRight.convertToRobotRelative(frontRight.getAngle()));
		SmartDashboard.putNumber("Corrected angle FL", frontLeft.convertToRobotRelative(frontLeft.getAngle()));
		SmartDashboard.putNumber("Corrected angle BR", backRight.convertToRobotRelative(backRight.getAngle()));
		SmartDashboard.putNumber("Corrected angle BL", backLeft.convertToRobotRelative(backLeft.getAngle()));
	}

	void syncroDrive(double driveSpeed, double driveAngle, double twist) {
		if (Math.abs(twist) > 0.5) {
			if (twist > 0) {
				twist = (twist - 0.5)*2;
			} else if (twist < 0) {
				twist = (twist + 0.5)*2;
			}
			frontRight.control(-twist, 45);
			frontLeft.control(twist, 315);
			backRight.control(-twist, 315);
			backLeft.control(twist, 45);
		} else {
			frontRight.control(driveSpeed, driveAngle);
			frontLeft.control(driveSpeed, driveAngle);
			backRight.control(driveSpeed, driveAngle);
			backLeft.control(driveSpeed, driveAngle);
		}

		// steerFrontRight.set(1);

		SmartDashboard.putNumber("front right encoder: ", frontRight.getAngle());
		SmartDashboard.putNumber("front left encoder: ", frontLeft.getAngle());
		SmartDashboard.putNumber("back right encoder: ", backRight.getAngle());
		SmartDashboard.putNumber("back left encoder: ", backLeft.getAngle());

		SmartDashboard.putNumber("Corrected angle FR", frontRight.convertToRobotRelative(frontRight.getAngle()));
		SmartDashboard.putNumber("Corrected angle FL", frontLeft.convertToRobotRelative(frontLeft.getAngle()));
		SmartDashboard.putNumber("Corrected angle BR", backRight.convertToRobotRelative(backRight.getAngle()));
		SmartDashboard.putNumber("Corrected angle BL", backLeft.convertToRobotRelative(backLeft.getAngle()));
	}


//odnt use this method
void translateAndRotate(double joystickX, double joystickY, double joystickAngle, double gyroReading, double targetDirection, double turnMagnitude) {
		//input: left joystick x, left joystick y, left joystick angle, current gyro reading(indegrees), the angle of the right joystick,
		//the magnitude of the right joystick

	//turns the gyro into a 0-360 range -- easier to work with
	SmartDashboard.putNumber("original gyro", gyroReading);
	double gyroValue = (Math.abs((360 * (((int)(gyroReading / 360)) + 1))) + gyroReading) % 360;

	//initializing the main variables
	double jsX = joystickX;
	double jsY = -joystickY;
	double driveDirection = ((joystickAngle - gyroValue) + 360) % 360; // will be 0 to 360
	double targetAngle = targetDirection - gyroValue; //will be -360 to 360
	double turnSpeed = turnMagnitude;
	double x = ROBOT_X_IN_CM;
	double y = ROBOT_Y_IN_CM;

	SmartDashboard.putNumber("gyro", gyroValue);

	SmartDashboard.putNumber("LeftJS x", jsX);
	SmartDashboard.putNumber("LeftJS y", jsY);
	SmartDashboard.putNumber("DriveDirection", driveDirection);
	
	if (targetAngle < 0) targetAngle += 360;
	targetAngle -= 180;

	if (Math.abs(targetAngle) >= 45) {
		if (targetAngle < -45) {
			targetAngle = -45;
		}
		if (targetAngle > 45) {
			targetAngle = 45;
		}
	}
	SmartDashboard.putNumber("targetAngle", targetAngle);


	//distance from the center of the robot to a module
	double w = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

	//calculations used later, logical name
	double c = w * Math.sin(Math.toRadians(45 + targetAngle));
	double d = w * Math.cos(Math.toRadians(45 + targetAngle));

	double translatedXModule1 = jsX + c;
	double translatedXModule2 = jsX + d;
	double translatedXModule3 = jsX - c;
	double translatedXModule4 = jsX - d;

	double translatedYModule1 = jsY + d;
	double translatedYModule2 = jsY + c;
	double translatedYModule3 = jsY - d;
	double translatedYModule4 = jsY - c;

	double xModule1 = x/2;
	double xModule2 = -x/2;
	double xModule3 = -x/2;
	double xModule4 = x/2;

	double yModule1 = y/2;
	double yModule2 = y/2;
	double yModule3 = -y/2;
	double yModule4 = -y/2;

	double length1 = Math.sqrt(Math.pow(translatedXModule1 - xModule1, 2) + Math.pow(translatedYModule1 - yModule1, 2));
	double length2 = Math.sqrt(Math.pow(translatedXModule2 - xModule2, 2) + Math.pow(translatedYModule2 - yModule2, 2));
	double length3 = Math.sqrt(Math.pow(translatedXModule3 - xModule3, 2) + Math.pow(translatedYModule3 - yModule3, 2));
	double length4 = Math.sqrt(Math.pow(translatedXModule4 - xModule4, 2) + Math.pow(translatedYModule4 - yModule4, 2));

	double lengthAverage = (length1 + length2 + length3 + length4) / 4;

	double basePower = lengthAverage / Math.sqrt(Math.pow(jsX, 2) + Math.pow(jsY, 2));

	double wheelPower1 = length1 / basePower;
	double wheelPower2 = length2 / basePower;
	double wheelPower3 = length3 / basePower;
	double wheelPower4 = length4 / basePower;

	double wheelAngle1 = driveDirection + Math.atan((translatedYModule1 - yModule1) / (translatedXModule1 - xModule1));
	double wheelAngle2 = driveDirection + Math.atan((translatedYModule2 - yModule2) / (translatedXModule2 - xModule2));
	double wheelAngle3 = driveDirection + Math.atan((translatedYModule3 - yModule3) / (translatedXModule3 - xModule3));
	double wheelAngle4 = driveDirection + Math.atan((translatedYModule4 - yModule4) / (translatedXModule4 - xModule4));

	double maxPower = wheelPower1;
	if (wheelPower2 > maxPower) maxPower = wheelPower2;
	if (wheelPower3 > maxPower) maxPower = wheelPower3;
	if (wheelPower4 > maxPower) maxPower = wheelPower4;

	if (maxPower > 1) {
		double powerScale = 1 / maxPower;
		wheelPower1 *= powerScale;
		wheelPower2 *= powerScale;
		wheelPower3 *= powerScale;
		wheelPower4 *= powerScale;
	}

	frontRight.control(wheelPower1, wheelAngle1);
	frontLeft.control(wheelPower2, wheelAngle2);
	backLeft.control(wheelPower3, wheelAngle3);
	backRight.control(wheelPower4, wheelAngle4);

	SmartDashboard.putNumber("Angle 1", wheelAngle1);
	SmartDashboard.putNumber("Angle 2", wheelAngle2);
	SmartDashboard.putNumber("Angle 3", wheelAngle3);
	SmartDashboard.putNumber("Angle 4", wheelAngle4);

	SmartDashboard.putNumber("Power1", wheelPower1);
	SmartDashboard.putNumber("Power2", wheelPower2);
	SmartDashboard.putNumber("Power3", wheelPower3);
	SmartDashboard.putNumber("Power4", wheelPower4);
	SmartDashboard.putNumber("Power4", wheelPower4);
	//if (absRotateSpeed > 0.05) {
	//	rotateAngle = absRotateAngle - ((360 * ((int)(gyroValue / 360) + 1)) + gyroValue) % 360;
	//	rotateSpeed = absRotateSpeed;
	//} else if (Math.abs(unregulatedRotateSpeed) > 0.05) {
	//	rotateAngle = (unregulatedRotateSpeed + 1) * 180; 
	//	rotateSpeed = Math.abs(unregulatedRotateSpeed);
	//} else {
	//	rotateAngle = 0;
	//	rotateSpeed = 0;
	//}
	}

	void parkPosition() {
		frontRight.control(0, -45);
		frontLeft.control(0, 45);
		backLeft.control(0, -45);
		backRight.control(0, 45);
	}

	void tuningMode() {
		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());
	}
}