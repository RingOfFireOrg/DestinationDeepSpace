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
}