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

	public static final int ENCODER_ZERO_VALUE_FRONT_LEFT = 7;
	public static final int ENCODER_ZERO_VALUE_FRONT_RIGHT = 46;
	public static final int ENCODER_ZERO_VALUE_BACK_LEFT = 25;
	public static final int ENCODER_ZERO_VALUE_BACK_RIGHT = 179;

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

	SwerveModule frontLeft;
	SwerveModule frontRight;
	SwerveModule backLeft;
	SwerveModule backRight;

	SwerveDrive() {
		frontRight = new SwerveModule(new Jaguar(DRIVE_FRONT_RIGHT_MOTOR), new Talon(STEER_FRONT_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_FRONT_RIGHT), ENCODER_ZERO_VALUE_FRONT_RIGHT, 
				new Encoder(DRIVE_ENCODER_FRONT_RIGHT_A, DRIVE_ENCODER_FRONT_RIGHT_B, false, Encoder.EncodingType.k2X));
		frontLeft = new SwerveModule(new Jaguar(DRIVE_FRONT_LEFT_MOTOR), new Talon(STEER_FRONT_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_FRONT_LEFT), ENCODER_ZERO_VALUE_FRONT_LEFT,
				new Encoder(DRIVE_ENCODER_FRONT_LEFT_A, DRIVE_ENCODER_FRONT_LEFT_B, false, Encoder.EncodingType.k2X));
		backLeft = new SwerveModule(new Jaguar(DRIVE_BACK_LEFT_MOTOR), new Talon(STEER_BACK_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_BACK_LEFT), ENCODER_ZERO_VALUE_BACK_LEFT,
				new Encoder(DRIVE_ENCODER_BACK_LEFT_A, DRIVE_ENCODER_BACK_LEFT_B, false, Encoder.EncodingType.k2X));
		backRight = new SwerveModule(new Jaguar(DRIVE_BACK_RIGHT_MOTOR), new Talon(STEER_BACK_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_BACK_RIGHT), ENCODER_ZERO_VALUE_BACK_RIGHT,
				new Encoder(DRIVE_ENCODER_BACK_RIGHT_A, DRIVE_ENCODER_BACK_RIGHT_B, false, Encoder.EncodingType.k2X));
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
void translateAndRotate(double joystickX, double joystickY, double joystickAngle, double gyroReading, 
	double targetDirection, double turnMagnitude, double robotX, double robotY) {

	double jsX = joystickX;
	double jsY = -joystickY;
	double jsA = joystickAngle;
	double gyroValue = (Math.abs((360 * ((int)(gyroReading / 360) + 1)) + gyroReading)) % 360;
	double targetAngle = targetDirection - gyroValue; //will be -360 to 360
	double turnSpeed = turnMagnitude;
	double x = robotX;
	double y = robotY;

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


	double w = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
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

	double translateDistance = Math.sqrt(Math.pow(jsX, 2) + Math.pow(jsY, 2));



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
}