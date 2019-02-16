package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

	public enum robotFront{SWTCH_TO_CARGO, SWITCH_TO_HATCH, NO_SETTING_CHANGE}
	AHRS ahrs;
	double ahrsOffset;
	PID pidDrivingStraight;
	RotatingBuffer gyroRateBuffer;
	boolean isCargoFront;
	
	public SwerveDrive(){
		return SwerveDrive(true);
	}

	public SwerveDrive(boolean isCargoFront){
		setIsCargoFront(isCargoFront);
	}

	static SwerveModule frontRight = new SwerveModule(new Jaguar(RobotMap.DRIVE_FRONT_RIGHT_MOTOR), new Talon(RobotMap.STEER_FRONT_RIGHT_MOTOR),
		 new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_RIGHT), RobotMap.ENCODER_ZERO_VALUE_FRONT_RIGHT, 
		 new Encoder(RobotMap.DRIVE_ENCODER_FRONT_RIGHT_A, RobotMap.DRIVE_ENCODER_FRONT_RIGHT_B, false, Encoder.EncodingType.k2X), "FrontRight");
	static SwerveModule frontLeft = new SwerveModule(new Jaguar(RobotMap.DRIVE_FRONT_LEFT_MOTOR), new Talon(RobotMap.STEER_FRONT_LEFT_MOTOR),
		 new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_LEFT), RobotMap.ENCODER_ZERO_VALUE_FRONT_LEFT,
		 new Encoder(RobotMap.DRIVE_ENCODER_FRONT_LEFT_A, RobotMap.DRIVE_ENCODER_FRONT_LEFT_B, false, Encoder.EncodingType.k2X), "FrontLeft");
	static SwerveModule backLeft = new SwerveModule(new Jaguar(RobotMap.DRIVE_BACK_LEFT_MOTOR), new Talon(RobotMap.STEER_BACK_LEFT_MOTOR),
		 new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_LEFT), RobotMap.ENCODER_ZERO_VALUE_BACK_LEFT, 
		 new Encoder(RobotMap.DRIVE_ENCODER_BACK_LEFT_A, RobotMap.DRIVE_ENCODER_BACK_LEFT_B, false, Encoder.EncodingType.k2X), "BackLeft");
	static SwerveModule backRight = new SwerveModule(new Jaguar(RobotMap.DRIVE_BACK_RIGHT_MOTOR), new Talon(RobotMap.STEER_BACK_RIGHT_MOTOR),
		 new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_RIGHT), RobotMap.ENCODER_ZERO_VALUE_BACK_RIGHT,  
		 new Encoder(RobotMap.DRIVE_ENCODER_BACK_RIGHT_A, RobotMap.DRIVE_ENCODER_BACK_RIGHT_B, false, Encoder.EncodingType.k2X), "BackRight");


	boolean driveStraight = false;
	double translationAngle;

	void swerveInit(){
		ahrs = new AHRS(SerialPort.Port.kUSB);
		ahrs.reset();
		ahrsOffset = ahrs.getAngle();
		//7:40 pm 2/14 -- 0.0025, 0.000025, 0.05
		pidDrivingStraight = new PID(0.0025, 0.000025, 0);
		pidDrivingStraight.setOutputRange(-0.5, 0.5);
		translationAngle = ahrs.getAngle() - ahrsOffset;
		frontRight.resetModule();
		frontLeft.resetModule();
		backLeft.resetModule();
		backRight.resetModule();
		frontLeft.invertModule();
		backLeft.invertModule();
		SmartDashboard.putNumber("Version #", 5);
		gyroRateBuffer = new RotatingBuffer(5);
	
	}	

	robotFront translateAndRotate(double driveFieldTranslationX, double driveFieldTranslationY, double unregulatedTurning, double gyroReading, double fieldRelativeRobotDirection, double driveRobotTranslationX, double driveRobotTranslationY) {
		//turns the gyro into a 0-360 range -- easier to work with
		double gyroValueUnprocessed = gyroReading;
		double gyroValueProcessed = (Math.abs(((int)(gyroReading)) * 360) + gyroReading) % 360;

		//initializing the main variables
		double fieldRelativeX = driveFieldTranslationX;
		double fieldRelativeY = driveFieldTranslationY;
		double robotRelativeX = driveRobotTranslationX;
		double robotRelativeY = driveRobotTranslationY;
		double unregulatedRotationValue = unregulatedTurning;
		double absoluteFieldRelativeDirection = fieldRelativeRobotDirection;

		//Translation Modes -- field relative or robot relative
		double jsMag = Math.sqrt(Math.pow(fieldRelativeX, 2) + Math.pow(fieldRelativeY, 2));
		if (jsMag < RobotMap.TRANSLATION_DEADZONE) jsMag = 0;
		if (jsMag != 0) {
			double initialAngle;

			if (fieldRelativeX == 0) {
				if (fieldRelativeY > 0) {
					initialAngle = 90;
				} else {
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
			robotRelativeX = jsMag * Math.cos(Math.toRadians(processedAngle));	
			robotRelativeY = jsMag * Math.sin(Math.toRadians(processedAngle));
		} else if (Math.sqrt(Math.pow(robotRelativeX, 2) + Math.pow(robotRelativeY, 2)) < RobotMap.TRANSLATION_DEADZONE) {
			//if the field relative code didn't run, robot rel will still be set from its declaration, this rules out deadzones
			robotRelativeX = 0;
			robotRelativeY = 0;
		}

		//Rotation Modes -- absolute, unregulated, and none
		//gyro rate buffer updating
		gyroRateBuffer.add(ahrs.getRate());
		double rotationMagnitude;
		if (absoluteFieldRelativeDirection != -1) {
			if (absoluteFieldRelativeDirection < 0) absoluteFieldRelativeDirection += 360;
			/*
			if (absoluteFieldRelativeDirection > 337.5 || absoluteFieldRelativeDirection <= 22.5) {
				absoluteFieldRelativeDirection = 0;
			} else if (absoluteFieldRelativeDirection > 22.5 && absoluteFieldRelativeDirection <= 67.5) {
				absoluteFieldRelativeDirection = 45;
			} else if (absoluteFieldRelativeDirection > 67.5 && absoluteFieldRelativeDirection <= 110.5) {
				absoluteFieldRelativeDirection = 90;
			} else if (absoluteFieldRelativeDirection > 110.5 && absoluteFieldRelativeDirection <= 157.5) {
				absoluteFieldRelativeDirection = 135;
			} else if (absoluteFieldRelativeDirection > 157.5 && absoluteFieldRelativeDirection <= 202.5) {
				absoluteFieldRelativeDirection = 180;
			} else if (absoluteFieldRelativeDirection > 202.5 && absoluteFieldRelativeDirection <= 247.5) {
				absoluteFieldRelativeDirection = 225;
			} else if (absoluteFieldRelativeDirection > 247.5 && absoluteFieldRelativeDirection <= 292.5) {
				absoluteFieldRelativeDirection = 270;
			} else if (absoluteFieldRelativeDirection == 315) {
				absoluteFieldRelativeDirection = 315;
			}
			*/
			if (gyroValueProcessed > 180 && absoluteFieldRelativeDirection == 0) {
				absoluteFieldRelativeDirection = 360;
			}
			rotationMagnitude = (absoluteFieldRelativeDirection - gyroValueProcessed) * 0.005;
			if (Math.abs(absoluteFieldRelativeDirection - gyroValueProcessed) > 180) rotationMagnitude *= -1;
			driveStraight = false;
		} else if (unregulatedRotationValue > RobotMap.ROTATION_DEADZONE || unregulatedRotationValue < -RobotMap.ROTATION_DEADZONE) {
			rotationMagnitude = unregulatedRotationValue;
			driveStraight = false;
		} else if (Math.sqrt(Math.pow(robotRelativeX, 2) + Math.pow(robotRelativeY, 2)) > RobotMap.TRANSLATION_DEADZONE * 0.75 && Math.abs(gyroRateBuffer.getAverage()) < 3) {
			//no turning methods -- goes straight
			if (driveStraight == false) {
				driveStraight = true;
				translationAngle = gyroValueUnprocessed;
				pidDrivingStraight.reset();
			}
			pidDrivingStraight.setError(gyroValueUnprocessed - translationAngle);
			pidDrivingStraight.update();
			rotationMagnitude = -pidDrivingStraight.getOutput();
			SmartDashboard.putNumber("translationAngle", translationAngle);
			SmartDashboard.putNumber("DSEC - ", -pidDrivingStraight.getOutput());
		} else {
			rotationMagnitude = 0;
		}
		if (rotationMagnitude > 1) rotationMagnitude = 1;
		if (rotationMagnitude < -1) rotationMagnitude = -1;

		SmartDashboard.putNumber("gyroRate", ahrs.getRate());
		

		//Vector math to combine the translation and the rotation values
		//adding the various cartesian points for the end of the vectors
		double xWithRotation = robotRelativeX + rotationMagnitude;
		double xWithoutRotation = robotRelativeX - rotationMagnitude;
		double yWithRotation = robotRelativeY + rotationMagnitude;
		double yWithoutRotation = robotRelativeY - rotationMagnitude;

		//Constructing the arrays to be used to determine outcomes for each wheel
		double wheelX[] = new double[4]; //the x value of the wheels vector
		double wheelY[] = new double[4]; //the y value of the wheels vector

		double wheelSpeed[] = new double[4]; //the speed that will be assigned to the wheels output
		double wheelAngle[] = new double[4]; // the angle that will be assigned to the modules output
	
		//individually processes each wheel -- determines speed and angle
		for (int i = 0 ; i < 4 ; i ++) {

			//for each module, the turn vectors will extend in a different direction
			if(i == 0 || i == 1) {
				wheelX[i] = xWithRotation;
			} else {
				wheelX[i] = xWithoutRotation;
			}

			if(i == 0 || i == 3) {
				wheelY[i] = yWithoutRotation;
			} else {
				wheelY[i] = yWithRotation;
			}

			//the wheels speed is just the distance from the end of its added vectors and the wheels center
			wheelSpeed[i] = Math.sqrt(Math.pow(wheelX[i], 2) + Math.pow(wheelY[i], 2));
			//the angle is the interior angle of the formed triangle
			wheelAngle[i] = Math.toDegrees(Math.atan(wheelX[i] / wheelY[i]));

			//The math only allows for directions in 2 quadrants, have to reassign values to gain the 2nd and 3rd quadrants
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

			//math is done assuming clockwise, wheel outputs are counterclockwise
			wheelAngle[i] *= -1;

			//makes all angles positive -- if negative will make it a positive co-terminal angle
			if (wheelAngle[i] < 0) {
				wheelAngle[i] += 360;
			}
		}
	
		//assures that no wheel is given a speed higher than 1 -- if so, will divide all speeds by the highest speed
		double maxSpeed = wheelSpeed[0];
		if (wheelSpeed[1] > maxSpeed) {maxSpeed = wheelSpeed[1];}
		if (wheelSpeed[2] > maxSpeed) {maxSpeed = wheelSpeed[2];}
		if (wheelSpeed[3] > maxSpeed) {maxSpeed = wheelSpeed[3];}
		if (maxSpeed > 1) {
			for (int i = 0 ; i < 4 ; i ++) {
				wheelSpeed[i] /= maxSpeed;
			}
		}
		
		//sets all modules to the calculated speed and angle
		frontRight.control(wheelSpeed[0], wheelAngle[0]);
		frontLeft.control(wheelSpeed[1], wheelAngle[1]);
		backLeft.control(wheelSpeed[2], wheelAngle[2]);
		backRight.control(wheelSpeed[3], wheelAngle[3]);

		//reads out the raw angles, processed angles, speed, and gyro
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

		SmartDashboard.putNumber("Gyro 0-360", gyroValueProcessed);

		if(){ //buttonpress
			front = SWTCH_TO_CARGO;
		} else if(){ //buttonpress
			front = SWITCH_TO_HATCH;
		} else{
			front = NO_SETTING_CHANGE;
		}
		
		return front;
	}

	void parkPosition() {
		//can be activated to give the robot increased traction when stopped
		frontRight.control(0, -45);
		frontLeft.control(0, 45);
		backLeft.control(0, -45);
		backRight.control(0, 45);
	}

	void tuningMode(JoystickButton buttonFR, JoystickButton buttonFL, JoystickButton buttonBL, JoystickButton buttonBR) {
		//used to tune the modules and their zero values
		/*
		if (buttonFR.get()) {
			frontRight.control(0.1, 0);
		} else {
			frontRight.stop();
		}
		if (buttonFL.get()) {
			frontLeft.control(0.1, 0);
		} else {
			frontLeft.stop();
		}
		if (buttonBR.get()) {
			backRight.control(0.1, 0);
		} else {
			backRight.stop();
		}
		if (buttonBL.get()) {
			backLeft.control(0.1, 0);
		} else {
			backLeft.stop();
		}
		*/

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());
	}

	void syncroDrive(double driveSpeed, double driveAngle, double twist, double gyroReading) {
		//not field relative yet -- sitll needs work
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

		SmartDashboard.putNumber("front right encoder: ", frontRight.getAngle());
		SmartDashboard.putNumber("front left encoder: ", frontLeft.getAngle());
		SmartDashboard.putNumber("back right encoder: ", backRight.getAngle());
		SmartDashboard.putNumber("back left encoder: ", backLeft.getAngle());

		SmartDashboard.putNumber("Corrected angle FR", frontRight.convertToRobotRelative(frontRight.getAngle()));
		SmartDashboard.putNumber("Corrected angle FL", frontLeft.convertToRobotRelative(frontLeft.getAngle()));
		SmartDashboard.putNumber("Corrected angle BR", backRight.convertToRobotRelative(backRight.getAngle()));
		SmartDashboard.putNumber("Corrected angle BL", backLeft.convertToRobotRelative(backLeft.getAngle()));
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

	boolean getIsCargoFront(){
		return this.isCargoFront;
	}

	void setIsCargoFront(boolean isCargoFront){
		this.isCargoFront = isCargoFront;
	}

	

}