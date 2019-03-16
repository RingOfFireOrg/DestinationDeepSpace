package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

	enum selectiveSwerveDriveModes {
		ROBOT_UNREGULATED, ROBOT_ABSOLUTE, FIELD_UNREGULATED, FIELD_ABSOLUTE
	}

	AHRS ahrs;
	double ahrsOffset;
	PID pidDrivingStraight;
	RotatingBuffer gyroRateBuffer;
	boolean driveStraight = false;
	double translationAngle;
	boolean isCargoFront = true;
	private selectiveSwerveDriveModes selectiveSwerveDriveMode;

	private static SwerveDrive swerveDrive;

	static SwerveModule frontRight = new SwerveModule(new TalonSRX(RobotMap.DRIVE_FRONT_RIGHT_MOTOR),
			new VictorSPX(RobotMap.STEER_FRONT_RIGHT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_RIGHT),
			RobotMap.ENCODER_ZERO_VALUE_FRONT_RIGHT, new Encoder(RobotMap.DRIVE_ENCODER_FRONT_RIGHT_A,
					RobotMap.DRIVE_ENCODER_FRONT_RIGHT_B, false, Encoder.EncodingType.k2X),
			"FrontRight");
	static SwerveModule frontLeft = new SwerveModule(new TalonSRX(RobotMap.DRIVE_FRONT_LEFT_MOTOR),
			new VictorSPX(RobotMap.STEER_FRONT_LEFT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_LEFT),
			RobotMap.ENCODER_ZERO_VALUE_FRONT_LEFT, new Encoder(RobotMap.DRIVE_ENCODER_FRONT_LEFT_A,
					RobotMap.DRIVE_ENCODER_FRONT_LEFT_B, false, Encoder.EncodingType.k2X),
			"FrontLeft");
	static SwerveModule backLeft = new SwerveModule(new TalonSRX(RobotMap.DRIVE_BACK_LEFT_MOTOR),
			new VictorSPX(RobotMap.STEER_BACK_LEFT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_LEFT),
			RobotMap.ENCODER_ZERO_VALUE_BACK_LEFT, new Encoder(RobotMap.DRIVE_ENCODER_BACK_LEFT_A,
					RobotMap.DRIVE_ENCODER_BACK_LEFT_B, false, Encoder.EncodingType.k2X),
			"BackLeft");
	static SwerveModule backRight = new SwerveModule(new TalonSRX(RobotMap.DRIVE_BACK_RIGHT_MOTOR),
			new VictorSPX(RobotMap.STEER_BACK_RIGHT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_RIGHT),
			RobotMap.ENCODER_ZERO_VALUE_BACK_RIGHT, new Encoder(RobotMap.DRIVE_ENCODER_BACK_RIGHT_A,
					RobotMap.DRIVE_ENCODER_BACK_RIGHT_B, false, Encoder.EncodingType.k2X),
			"BackRight");

	protected SwerveDrive(AHRS ahrs) {
		this.ahrs = ahrs;
		ahrsOffset = ahrs.getAngle();
		translationAngle = 0;

		pidDrivingStraight = new PID(0.0025, 0.000025, 0);
		pidDrivingStraight.setOutputRange(-0.5, 0.5);

		reset();
		gyroRateBuffer = new RotatingBuffer(5);
	}

	public static SwerveDrive getInstance(AHRS ahrs) {
		if (swerveDrive == null) {
			swerveDrive = new SwerveDrive(ahrs);
		}
		return swerveDrive;
	}

	void translateAndRotate(double driveFieldTranslationX, double driveFieldTranslationY, double unregulatedTurning, double fieldRelativeRobotDirection, double driveRobotTranslationX,
			double driveRobotTranslationY) {
		// turns the gyro into a 0-360 range -- easier to work with
		double gyroValueUnprocessed = ahrs.getAngle() - this.ahrsOffset;
		double gyroValueProcessed = (Math.abs(((int) (gyroValueUnprocessed)) * 360) + gyroValueUnprocessed) % 360;

		// initializing the main variables
		double fieldRelativeX = driveFieldTranslationX;
		double fieldRelativeY = driveFieldTranslationY;
		double robotRelativeX;
		double robotRelativeY;

		if (isCargoFront) {
			robotRelativeX = driveRobotTranslationX;
			robotRelativeY = driveRobotTranslationY;
		} else {
			robotRelativeX = -driveRobotTranslationY;
			robotRelativeY = driveRobotTranslationX;
		}

		double unregulatedRotationValue = unregulatedTurning;
		double absoluteFieldRelativeDirection = fieldRelativeRobotDirection;

		// Translation Modes -- field relative or robot relative
		double fieldTransMag = Math.sqrt(Math.pow(fieldRelativeX, 2) + Math.pow(fieldRelativeY, 2));

		// if (fieldTransMag < RobotMap.TRANSLATION_DEADZONE) {
		// 	fieldTransMag = 0;
		// }
			
		if (fieldTransMag != 0) {
			double initialAngle;

			//converting the field translations to an angle
			if (fieldRelativeX == 0) {
				if (fieldRelativeY > 0) {
					//eliminates tan asymptotes
					initialAngle = 90;
				} else {
					//eliminates tan asymptotes
					initialAngle = -90;
				}
			} else {
				initialAngle = Math.toDegrees(Math.atan(fieldRelativeY / fieldRelativeX));
			}

			if (fieldRelativeX < 0) {
				if (fieldRelativeY > 0) {
					initialAngle += 180;
				} else {
					initialAngle -= 180;
				}
			}

			double processedAngle = initialAngle + gyroValueProcessed;
			robotRelativeX = fieldTransMag * Math.cos(Math.toRadians(processedAngle));
			robotRelativeY = fieldTransMag * Math.sin(Math.toRadians(processedAngle));
		} //else if (Math.sqrt(Math.pow(robotRelativeX, 2) + Math.pow(robotRelativeY, 2)) < RobotMap.TRANSLATION_DEADZONE) {
		// 	// if the field relative code didn't run, robot rel will still be set from its
		// 	// declaration, this rules out deadzones
		// 	robotRelativeX = 0;
		// 	robotRelativeY = 0;
		// }

		// Rotation Modes -- absolute, unregulated, and none
		// gyro rate buffer updating
		gyroRateBuffer.add(ahrs.getRate());
		double rotationMagnitude;
		if (absoluteFieldRelativeDirection != -1) {
			if (absoluteFieldRelativeDirection < 0)
				absoluteFieldRelativeDirection += 360;
			if (gyroValueProcessed > 180 && absoluteFieldRelativeDirection == 0) {
				absoluteFieldRelativeDirection = 360;
			}
			rotationMagnitude = (absoluteFieldRelativeDirection - gyroValueProcessed) * 0.005;
			if (Math.abs(absoluteFieldRelativeDirection - gyroValueProcessed) > 180)
				rotationMagnitude *= -1;
			driveStraight = false;
		} else if (unregulatedRotationValue > RobotMap.ROTATION_DEADZONE
				|| unregulatedRotationValue < -RobotMap.ROTATION_DEADZONE) {
			rotationMagnitude = unregulatedRotationValue;
			driveStraight = false;
		} else if (Math.sqrt(Math.pow(robotRelativeX, 2) + Math.pow(robotRelativeY, 2)) > RobotMap.TRANSLATION_DEADZONE
				* 0.75 && Math.abs(gyroRateBuffer.getAverage()) < 3) {
			// no turning methods -- goes straight
			if (driveStraight == false) {
				driveStraight = true;
				translationAngle = gyroValueUnprocessed;
				pidDrivingStraight.reset();
			}
			pidDrivingStraight.setError(gyroValueUnprocessed - translationAngle);
			pidDrivingStraight.update();
			rotationMagnitude = -pidDrivingStraight.getOutput();
			// SmartDashboard.putNumber("translationAngle", translationAngle);
			// SmartDashboard.putNumber("DSEC - ", -pidDrivingStraight.getOutput());
		} else {
			rotationMagnitude = 0;
		}
		if (rotationMagnitude > 1)
			rotationMagnitude = 1;
		if (rotationMagnitude < -1)
			rotationMagnitude = -1;

		// SmartDashboard.putNumber("gyroRate", ahrs.getRate());

		// Vector math to combine the translation and the rotation values
		// adding the various cartesian points for the end of the vectors
		double xWithRotation = robotRelativeX + rotationMagnitude;
		double xWithoutRotation = robotRelativeX - rotationMagnitude;
		double yWithRotation = robotRelativeY + rotationMagnitude;
		double yWithoutRotation = robotRelativeY - rotationMagnitude;

		// Constructing the arrays to be used to determine outcomes for each wheel
		double wheelX[] = new double[4]; // the x value of the wheels vector
		double wheelY[] = new double[4]; // the y value of the wheels vector

		double wheelSpeed[] = new double[4]; // the speed that will be assigned to the wheels output
		double wheelAngle[] = new double[4]; // the angle that will be assigned to the modules output

		// individually processes each wheel -- determines speed and angle
		for (int i = 0; i < 4; i++) {

			// for each module, the turn vectors will extend in a different direction
			if (i == 0 || i == 1) {
				wheelX[i] = xWithRotation;
			} else {
				wheelX[i] = xWithoutRotation;
			}

			if (i == 0 || i == 3) {
				wheelY[i] = yWithoutRotation;
			} else {
				wheelY[i] = yWithRotation;
			}

			// the wheels speed is just the distance from the end of its added vectors and
			// the wheels center
			wheelSpeed[i] = Math.sqrt(Math.pow(wheelX[i], 2) + Math.pow(wheelY[i], 2));
			// the angle is the interior angle of the formed triangle
			wheelAngle[i] = Math.toDegrees(Math.atan(wheelX[i] / wheelY[i]));

			// The math only allows for directions in 2 quadrants, have to reassign values
			// to gain the 2nd and 3rd quadrants
			if (wheelX[i] >= 0) {
				if (wheelY[i] >= 0) {
					// already in Q1
				} else {
					// shift to Q4
					wheelAngle[i] += 180;
				}
			} else {
				if (wheelY[i] >= 0) {
					// shift to Q2
				} else {
					// shift to Q3
					wheelAngle[i] -= 180;
				}
			}

			// math is done assuming clockwise, wheel outputs are counterclockwise
			wheelAngle[i] *= -1;

			// makes all angles positive -- if negative will make it a positive co-terminal
			// angle
			if (wheelAngle[i] < 0) {
				wheelAngle[i] += 360;
			}
		}

		// assures that no wheel is given a speed higher than 1 -- if so, will divide
		// all speeds by the highest speed
		double maxSpeed = wheelSpeed[0];
		if (wheelSpeed[1] > maxSpeed) {
			maxSpeed = wheelSpeed[1];
		}
		if (wheelSpeed[2] > maxSpeed) {
			maxSpeed = wheelSpeed[2];
		}
		if (wheelSpeed[3] > maxSpeed) {
			maxSpeed = wheelSpeed[3];
		}
		if (maxSpeed > 1) {
			for (int i = 0; i < 4; i++) {
				wheelSpeed[i] /= maxSpeed;
			}
		}

		// sets all modules to the calculated speed and angle
		frontRight.control(wheelSpeed[0], wheelAngle[0]);
		frontLeft.control(wheelSpeed[1], wheelAngle[1]);
		backLeft.control(wheelSpeed[2], wheelAngle[2]);
		backRight.control(wheelSpeed[3], wheelAngle[3]);

		// reads out the raw angles, processed angles, speed, and gyro
		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());

		// SmartDashboard.putNumber("FR Speed", wheelSpeed[0]);
		// SmartDashboard.putNumber("FL Speed", wheelSpeed[1]);
		// SmartDashboard.putNumber("BL Speed", wheelSpeed[2]);
		// SmartDashboard.putNumber("BR Speed", wheelSpeed[3]);

		// SmartDashboard.putNumber("FR Angle", wheelAngle[0]);
		// SmartDashboard.putNumber("FL Angle", wheelAngle[1]);
		// SmartDashboard.putNumber("BL Angle", wheelAngle[2]);
		// SmartDashboard.putNumber("BR Angle", wheelAngle[3]);

		SmartDashboard.putNumber("Gyro 0-360", gyroValueProcessed);
	}

	void selectiveTranslateAndRotate(selectiveSwerveDriveModes selectiveSwerveDriveMode, double turnInput,
			double translateXInput, double translateYInput) {
		this.selectiveSwerveDriveMode = selectiveSwerveDriveMode;
		switch (this.selectiveSwerveDriveMode) {
		case ROBOT_UNREGULATED:
			translateAndRotate(0, 0, turnInput, -1, translateXInput, translateYInput);
			break;
		case ROBOT_ABSOLUTE:
			translateAndRotate(0, 0, 0, turnInput, translateXInput, translateYInput);
			break;
		case FIELD_UNREGULATED:
			translateAndRotate(translateXInput, translateYInput, turnInput, -1, 0, 0);
			break;
		case FIELD_ABSOLUTE:
			translateAndRotate(translateXInput, translateYInput, 0, turnInput, 0, 0);
			break;
		}
	}

	void setAHRSOffset(double ahrsOffset) {
		this.ahrsOffset = ahrsOffset;
	}

	void parkPosition() {
		// can be activated to give the robot increased traction when stopped
		frontRight.control(0, -45);
		frontLeft.control(0, 45);
		backLeft.control(0, -45);
		backRight.control(0, 45);
	}

	void syncroDrive(double driveSpeed, double driveAngle, double twist, double gyroReading) {
		// not field relative yet -- sitll needs work
		driveAngle += gyroReading;

		if (Math.abs(twist) > 0.5) {
			if (twist > 0) {
				twist = (twist - 0.5) * 2;
			} else if (twist < 0) {
				twist = (twist + 0.5) * 2;
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

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());
	}

	void tuningMode() {
		// used to tune the modules and their zero values

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());
	}

	void setRobotFrontToCargo() {
		isCargoFront = true;
	}

	void setRobotFrontToHatch() {
		isCargoFront = false;
	}

	double squareWithSignReturn(double inputReading) {
		return Math.signum(inputReading) * inputReading * inputReading;
	}

	double degToInches(double degrees) {
		double wheelRotations = degrees / 360;

		return RobotMap.WHEEL_CIRCUMFERENCE * wheelRotations;
	}

	double inchesToDeg(double inches) {
		double wheelRotations = inches / RobotMap.WHEEL_CIRCUMFERENCE;

		return wheelRotations * 360;
	}

	protected void reset() {
		frontRight.resetModule();
		frontLeft.resetModule();
		backLeft.resetModule();
		backRight.resetModule();
		frontLeft.invertModule();
		backLeft.invertModule();
	}

	// For testing purposes
	void individualModuleControl() {
		frontRight.setDriveSpeed(0);
		frontLeft.setDriveSpeed(0);
		backLeft.setDriveSpeed(0);
		backRight.setDriveSpeed(0);
		frontRight.setSteerSpeed(0);
		frontLeft.setSteerSpeed(0);
		backLeft.setSteerSpeed(0);
		backRight.setSteerSpeed(0);

		switch ((int) (SmartDashboard.getNumber("IMC", 0))) {
		case 0:
			frontRight.setDriveSpeed(0.3);
			break;
		case 1:
			frontLeft.setDriveSpeed(0.3);
			break;
		case 2:
			backLeft.setDriveSpeed(0.3);
			break;
		case 3:
			backRight.setDriveSpeed(0.3);
			break;
		case 4:
			frontRight.setSteerSpeed(0.3);
			break;
		case 5:
			frontLeft.setSteerSpeed(0.3);
			break;
		case 6:
			backLeft.setSteerSpeed(0.3);
			break;
		case 7:
			backRight.setSteerSpeed(0.3);
			break;
		default:
			break;
		}

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());

		// SmartDashboard.putNumber("Corrected angle FR",
		// frontRight.convertToRobotRelative(frontRight.getAngle()));
		// SmartDashboard.putNumber("Corrected angle FL",
		// frontLeft.convertToRobotRelative(frontLeft.getAngle()));
		// SmartDashboard.putNumber("Corrected angle BR",
		// backRight.convertToRobotRelative(backRight.getAngle()));
		// SmartDashboard.putNumber("Corrected angle BL",
		// backLeft.convertToRobotRelative(backLeft.getAngle()));
	}

	void testSwerveModule(boolean isFront, boolean isLeft, double driveSpeed, double steerSpeed) {
		if (isFront && isLeft) {
			frontLeft.setDriveSpeed(driveSpeed);
			frontLeft.setSteerSpeed(steerSpeed);
		} else if (isFront && !isLeft) {
			frontRight.setDriveSpeed(driveSpeed);
			frontRight.setSteerSpeed(steerSpeed);
		} else if (!isFront && isLeft) {
			backLeft.setDriveSpeed(driveSpeed);
			backLeft.setSteerSpeed(steerSpeed);
		} else if (!isFront && !isLeft) {
			backRight.setDriveSpeed(driveSpeed);
			backRight.setSteerSpeed(steerSpeed);
		}
	}
}