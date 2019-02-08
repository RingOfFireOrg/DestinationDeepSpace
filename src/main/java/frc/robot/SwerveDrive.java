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

	SwerveModule frontLeft;
	SwerveModule frontRight;
	SwerveModule backLeft;
	SwerveModule backRight;

	AHRS ahrs;
	double ahrsOffset;



		Encoder driveFrontRight = new Encoder(RobotMap.DRIVE_ENCODER_FRONT_RIGHT_A, RobotMap.DRIVE_ENCODER_FRONT_RIGHT_B, false, Encoder.EncodingType.k2X);
		Encoder driveFrontLeft = new Encoder(RobotMap.DRIVE_ENCODER_FRONT_LEFT_A, RobotMap.DRIVE_ENCODER_FRONT_LEFT_B, false, Encoder.EncodingType.k2X);
		Encoder driveBackLeft = new Encoder(RobotMap.DRIVE_ENCODER_BACK_LEFT_A, RobotMap.DRIVE_ENCODER_BACK_LEFT_B, false, Encoder.EncodingType.k2X);
		Encoder driveBackRight = new Encoder(RobotMap.DRIVE_ENCODER_BACK_RIGHT_A, RobotMap.DRIVE_ENCODER_BACK_RIGHT_B, false, Encoder.EncodingType.k2X);

		frontRight = new SwerveModule(new Jaguar(RobotMap.DRIVE_FRONT_RIGHT_MOTOR), new Talon(RobotMap.STEER_FRONT_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_RIGHT), RobotMap.ENCODER_ZERO_VALUE_FRONT_RIGHT, driveFrontRight, "FrontRight");
		frontLeft = new SwerveModule(new Jaguar(RobotMap.DRIVE_FRONT_LEFT_MOTOR), new Talon(RobotMap.STEER_FRONT_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_LEFT), RobotMap.ENCODER_ZERO_VALUE_FRONT_LEFT, driveFrontLeft, "FrontLeft");
		backLeft = new SwerveModule(new Jaguar(RobotMap.DRIVE_BACK_LEFT_MOTOR), new Talon(RobotMap.STEER_BACK_LEFT_MOTOR),
				new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_LEFT), RobotMap.ENCODER_ZERO_VALUE_BACK_LEFT, driveBackLeft, "BackLeft");
		backRight = new SwerveModule(new Jaguar(RobotMap.DRIVE_BACK_RIGHT_MOTOR), new Talon(RobotMap.STEER_BACK_RIGHT_MOTOR),
				new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_RIGHT), RobotMap.ENCODER_ZERO_VALUE_BACK_RIGHT, driveBackRight, "BackRight");
		
	
	//	ahrs.reset();
	//	ahrsOffset = ahrs.getAngle();


	static void runSwerve(Joystick left, Joystick right, JoystickButton rightButton1, JoystickButton rightButton7) {

		Joystick leftStick = left;
		Joystick rightStick = right;
		JoystickButton rightTrigger = rightButton1;
		JoystickButton tuningActivation = rightButton7;

		int driveMode = 0;

		double speed = Math.pow(leftStick.getMagnitude(), 2);
		double leftDirection = leftStick.getDirectionDegrees() * -1;
		double leftX = leftStick.getX();
		double leftY = leftStick.getY();

		double rightDirection = rightStick.getDirectionDegrees();
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
				translateAndRotate(leftX, leftY, twist, ahrs.getAngle() - ahrsOffset, rightDirection, rightMagnitude);
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

	static void individualModuleControl(boolean buttonfr, boolean buttonfl, boolean buttonbr, boolean buttonbl) {
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

	static void syncroDrive(double driveSpeed, double driveAngle, double twist, double gyroReading) {

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

	static void translateAndRotate(double driveJoystickX, double driveJoystickY, double rightTwist, double gyroReading, double rightJoystickDirection, double rightMagnitude) {

		//turns the gyro into a 0-360 range -- easier to work with
		//SmartDashboard.putNumber("original gyro", gyroReading);
		double gyroValue = (Math.abs(((int)(gyroReading)) * 360) + gyroReading) % 360;
		//initializing the main variables
		double jsX = driveJoystickX;
		double jsY = -driveJoystickY;
		double twist = rightTwist;
		double rightMag = rightMagnitude;
		double rightDirection = rightJoystickDirection;

		SmartDashboard.putNumber("RtJSDir", rightDirection);

		if (rightMag > 0.3) {
			if (rightDirection < 0) rightDirection += 360;
			//if (Math.abs(rightDirection % 90 - 45) > 40) rightDirection += (Math.abs(rightDirection % 90) / (rightDirection % 90)(Math.abs((rightDirection % 90) - 45) - 40)
			/* if (rightDirection < 5 || rightDirection > 355) rightDirection = 0;
			if (rightDirection < 95 || rightDirection > 85) rightDirection = 90;
			if (rightDirection < 185 || rightDirection > 175) rightDirection = 180;
			if (rightDirection < 275 || rightDirection > 265) rightDirection = 270;
			*/
			twist = ((rightDirection - gyroValue) / 100) * Math.pow((rightMag / 1.5), 2);
			if (twist > 1) twist = 1;
			if (twist < -1) twist = -1;
			if (Math.abs(rightDirection - gyroValue) > 180) twist *= -1;
		}

		//convert to field relative
		double jsMag = Math.sqrt(Math.pow(jsX, 2) + Math.pow(jsY, 2));

		if (jsMag < 0.1) jsMag = 0;

		double initialAngle = Math.toDegrees(Math.atan(jsY / jsX));
		 if (jsX < 0) {
			if (jsY > 0) {
				initialAngle += 180;
			} else {
				initialAngle -= 180;
			}
		}
		double processedAngle = initialAngle + gyroValue;
		double robotRelativeX = jsMag * Math.cos(Math.toRadians(processedAngle));	
		double robotRelativeY = jsMag * Math.sin(Math.toRadians(processedAngle));
		
		SmartDashboard.putNumber("RelX", robotRelativeX);
		SmartDashboard.putNumber("RelY", robotRelativeY);
		SmartDashboard.putNumber("JSMAG", jsMag);
		SmartDashboard.putNumber("procAngle", processedAngle);
		SmartDashboard.putNumber("initAngle", initialAngle);

		
		double xWithTwist = robotRelativeX + twist;
		double xWithoutTwist = robotRelativeX - twist;
		double yWithTwist = robotRelativeY + twist;
		double yWithoutTwist = robotRelativeY - twist;
		

		/*
		double xWithTwist = robotRelativeX;
		double yWithoutTwist = robotRelativeY;
		double xWithoutTwist = robotRelativeX;
		double yWithTwist = robotRelativeY;
		*/

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

		SmartDashboard.putNumber("Gyro 0-360", gyroValue);
	}

	static void parkPosition() {
		frontRight.control(0, -45);
		frontLeft.control(0, 45);
		backLeft.control(0, -45);
		backRight.control(0, 45);
	}

	static void tuningMode() {
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