package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
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

	public static final int ENCODER_ZERO_VALUE_FRONT_RIGHT = 53;
	public static final int ENCODER_ZERO_VALUE_FRONT_LEFT = 16;
	public static final int ENCODER_ZERO_VALUE_BACK_LEFT = 46;
	public static final int ENCODER_ZERO_VALUE_BACK_RIGHT = 300;

	public static final int ENCODER_FRONT_RIGHT = 0;
	public static final int ENCODER_FRONT_LEFT = 1;
	public static final int ENCODER_BACK_LEFT = 2;
	public static final int ENCODER_BACK_RIGHT = 3;

	//need to get real numbers for drive encoders
	public static final int DRIVE_ENCODER_FRONT_RIGHT_A = 10;
	public static final int DRIVE_ENCODER_FRONT_RIGHT_B = 11;
	public static final int DRIVE_ENCODER_FRONT_LEFT_A = 12;
	public static final int DRIVE_ENCODER_FRONT_LEFT_B = 13;
	public static final int DRIVE_ENCODER_BACK_LEFT_A = 14;
	public static final int DRIVE_ENCODER_BACK_LEFT_B = 15;
	public static final int DRIVE_ENCODER_BACK_RIGHT_A = 16;
	public static final int DRIVE_ENCODER_BACK_RIGHT_B = 17;

	//dimensions of the robot in CM
	public static final int ROBOT_X_IN_CM = 51;
	public static final int ROBOT_Y_IN_CM = 51;


	SwerveModule frontLeft;
	SwerveModule frontRight;
	SwerveModule backLeft;
	SwerveModule backRight;

	AHRS ahrs;
	double ahrsOffset;

	SwerveDrive() {

		Encoder driveFrontRight = new Encoder(DRIVE_ENCODER_FRONT_RIGHT_A, DRIVE_ENCODER_FRONT_RIGHT_B, false, Encoder.EncodingType.k2X);
		Encoder driveFrontLeft = new Encoder(DRIVE_ENCODER_FRONT_LEFT_A, DRIVE_ENCODER_FRONT_LEFT_B, false, Encoder.EncodingType.k2X);
		Encoder driveBackLeft = new Encoder(DRIVE_ENCODER_BACK_LEFT_A, DRIVE_ENCODER_BACK_LEFT_B, false, Encoder.EncodingType.k2X);
		Encoder driveBackRight = new Encoder(DRIVE_ENCODER_BACK_RIGHT_A, DRIVE_ENCODER_BACK_RIGHT_B, false, Encoder.EncodingType.k2X);

		frontRight = new SwerveModule(new Jaguar(DRIVE_FRONT_RIGHT_MOTOR), new Talon(STEER_FRONT_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_FRONT_RIGHT), ENCODER_ZERO_VALUE_FRONT_RIGHT, driveFrontRight, "FrontRight");
		frontLeft = new SwerveModule(new Jaguar(DRIVE_FRONT_LEFT_MOTOR), new Talon(STEER_FRONT_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_FRONT_LEFT), ENCODER_ZERO_VALUE_FRONT_LEFT, driveFrontLeft, "FrontLeft");
		backLeft = new SwerveModule(new Jaguar(DRIVE_BACK_LEFT_MOTOR), new Talon(STEER_BACK_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_BACK_LEFT), ENCODER_ZERO_VALUE_BACK_LEFT, driveBackLeft, "BackLeft");
		backRight = new SwerveModule(new Jaguar(DRIVE_BACK_RIGHT_MOTOR), new Talon(STEER_BACK_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(ENCODER_BACK_RIGHT), ENCODER_ZERO_VALUE_BACK_RIGHT, driveBackRight, "BackRight");
		
		try {
			ahrs = new AHRS(SerialPort.Port.kUSB1);
		  } catch (RuntimeException ex) {
			  //DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		  }
		  
		ahrs.reset();
		ahrsOffset = ahrs.getAngle();

	}

	void runSwerve(Joystick left, Joystick right, JoystickButton rightButton1, JoystickButton rightButton7) {

		Joystick leftStick = left;
		Joystick rightStick = right;
		JoystickButton rightTrigger = rightButton1;
		JoystickButton tuningActivation = rightButton7;

		int driveMode = 0;

		double speed = Math.pow(leftStick.getMagnitude(), 2);
		double leftDirection = leftStick.getDirectionDegrees() * -1;
		double leftX = leftStick.getX();
		double leftY = leftStick.getY();

		double rightDirection = rightStick.getDirectionDegrees() * -1;
		double rightMagnitude = rightStick.getMagnitude();
		double twist = rightStick.getTwist();
		

		if (twist < 0) {
			twist = -Math.pow(twist, 2);
		} else {
			twist = Math.pow(twist, 2);
		}

		if (tuningActivation.get() == true) {
			driveMode = 1;
		} else { //if (rightStick.getMagnitude() > 0.3)// {
			driveMode = 3;
		} //else {
			//driveMode = 0;
		//}


		switch (driveMode) {
			case 0:
				//syncroDrive(speed, leftDirection, twist, ahrs.getAngle() - ahrsOffset);
				break;

			case 1:
				tuningMode();
				break;

			case 2:
				//translateAndRotate(leftX, leftY, leftDirection, ahrs.getAngle() - ahrsOffset, rightDirection, rightMagnitude);
				break;

			case 3:
				//syncroDrive(speed, leftDirection, twist, ahrs.getAngle() - ahrsOffset);
				translateAndRotate(leftX, leftY, twist, ahrs.getAngle() - ahrsOffset);
				break;
		
			default:
				break;
		}
		
		if (rightTrigger.get() == true) {
			ahrsOffset = ahrs.getAngle();
		}
			
			  
		SmartDashboard.putNumber("ahrs angle", ahrs.getAngle() - ahrsOffset);
		SmartDashboard.putNumber("Joystick output", leftDirection);
		SmartDashboard.putNumber("Joystick output speed", speed);
		
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

	void syncroDrive(double driveSpeed, double driveAngle, double twist, double gyroReading) {

		driveAngle += gyroReading;

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

	void translateAndRotate(double driveJoystickX, double driveJoystickY, double rightTwist, double gyroReading) {

		//turns the gyro into a 0-360 range -- easier to work with
		SmartDashboard.putNumber("original gyro", gyroReading);
		double gyroValue = (Math.abs((360 * (((int)(gyroReading / 360)) + 1))) + gyroReading) % 360;
		SmartDashboard.putNumber("tuned gyro", gyroValue);
		//initializing the main variables
		double jsX = driveJoystickX;
		double jsY = -driveJoystickY;
		double twist = rightTwist;

		double xWithTwist = jsX + twist;
		double xWithoutTwist = jsX - twist;
		double yWithTwist = jsY + twist;
		double yWithoutTwist = jsY - twist;

		double wheelX[] = new double[4];
		double wheelY[] = new double[4];

		double wheelSpeed[] = new double[4];
		double wheelAngle[] = new double[4];
	
		for (int i = 0 ; i < 4 ; i ++) {
			if(i == 0 || i == 1) {
				wheelX[i] = xWithTwist;
			} else {
				wheelX[i] = xWithoutTwist;
			}

			if(i == 0 || i == 3) {
				wheelY[i] = yWithoutTwist;
			} else {
				wheelY[i] = yWithTwist;
			}

			wheelSpeed[i] = Math.sqrt(Math.pow(wheelX[i], 2) + Math.pow(wheelY[i], 2));
			wheelAngle[i] = Math.toDegrees(Math.atan(wheelX[i] / wheelY[i]));

			if(wheelX[i] >= 0) {
				if (wheelY[i] >= 0) {
					//already in Q1
				} else {
					//shift to Q4
					wheelAngle[i] += 180;
				}
			} else {
				if (wheelY[i] >= 0) {
					//shift to Q2
				} else {
					//shift to Q3
					wheelAngle[i] -= 180;
				}
			}

			wheelAngle[i] *= -1;

			if (wheelAngle[i] < 0) {
				wheelAngle[i] += 360;
			}
		}
	
		double maxSpeed = wheelSpeed[0];
		if (wheelSpeed[1] > maxSpeed) {maxSpeed = wheelSpeed[1];}
		if (wheelSpeed[2] > maxSpeed) {maxSpeed = wheelSpeed[2];}
		if (wheelSpeed[3] > maxSpeed) {maxSpeed = wheelSpeed[3];}
		if (maxSpeed > 1) {
			for (int i = 0 ; i < 4 ; i ++) {
				wheelSpeed[i] /= maxSpeed;
			}
		}
		
		frontRight.control(wheelSpeed[0], wheelAngle[0]);
		frontLeft.control(wheelSpeed[1], wheelAngle[1]);
		backLeft.control(wheelSpeed[2], wheelAngle[2]);
		backRight.control(wheelSpeed[3], wheelAngle[3]);

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());

		SmartDashboard.putNumber("FR Speed", wheelSpeed[0]);
		SmartDashboard.putNumber("FL Speed", wheelSpeed[1]);
		SmartDashboard.putNumber("BL Speed", wheelSpeed[2]);
		SmartDashboard.putNumber("BR Speed", wheelSpeed[3]);

		SmartDashboard.putNumber("FR Angle", wheelAngle[0]);
		SmartDashboard.putNumber("FL Angle", wheelAngle[1]);
		SmartDashboard.putNumber("BL Angle", wheelAngle[2]);
		SmartDashboard.putNumber("BR Angle", wheelAngle[3]);
	}

	void parkPosition() {
		frontRight.control(0, -45);
		frontLeft.control(0, 45);
		backLeft.control(0, -45);
		backRight.control(0, 45);
	}

	void tuningMode() {
		//frontLeft.control(0, 0);
		//frontRight.control(0, 0);
		//backLeft.control(0, 0);
		//backRight.control(0, 0);
		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());
	}
}