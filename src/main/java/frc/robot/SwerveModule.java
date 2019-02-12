package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PIDController;
	   
//need to ifx
public class SwerveModule {
	Jaguar drive;
	Talon steer;
	AbsoluteAnalogEncoder turnEncoder;
	Encoder driveEncoder;
	double speed;
	double turnSpeed;
	double angleGoal;
	double currentAngle;
	double zeroValue;
	String moduleName;
	double optimizedSpeed;
	PID speedRegulation;
	double accumulatedGR = 0;
	

	//will need to make changes to the input --Encoder driveRotEncoder <-- add to constructor
	public SwerveModule(Jaguar driveMotor, Talon steerMotor, AbsoluteAnalogEncoder steerEncoder, double zeroValue, Encoder driveRotEncoder, String name) {
		this.zeroValue = zeroValue;
		drive = driveMotor;
		steer = steerMotor;
		turnEncoder = steerEncoder;
		moduleName = name;
		driveEncoder = driveRotEncoder;

		driveEncoder.reset();
		driveEncoder.setDistancePerPulse(18); //in degrees (360)/(20 pulses per rotation)

		speedRegulation = new PID(0, 0, 0);
		speedRegulation.setOutputRange(-1, 1);
		//speedRegulation = new PIDController(0.0001, 0, 0, driveEncoder, drive);
		//speedRegulation.setSetpoint(0);
		//speedRegulation.setContinuous();
		//speedRegulation.setInputRange(-31860, 31860);
		//speedRegulation.setOutputRange(-1.0, 1.0);
	}

	public double convertToWheelRelative(double wheelAngleGoal) {
		return ((wheelAngleGoal + zeroValue) + 720) % 360;
	}

	public double convertToRobotRelative(double wheelAngleGoal) {
		return ((wheelAngleGoal - zeroValue) + 720) % 360;
	}

	public double getAngle() {
		return turnEncoder.getAngle();
	}

	public double getRate() {
		return driveEncoder.getRate(); //in degrees per second
	}

	public void stop() {
		drive.set(0);
		steer.set(0);
	}

	public void setDriveSpeed(double drivePower) {
		accumulatedGR += drivePower;
		speedRegulation.setError((drivePower * 28000) - getRate());
		speedRegulation.update();
		optimizedSpeed = drivePower + speedRegulation.getOutput();
		if (optimizedSpeed > 1) optimizedSpeed = 1;
		if (optimizedSpeed < -1) optimizedSpeed = -1;
		drive.set(optimizedSpeed);
		/*
		SmartDashboard.putNumber("OS - " + moduleName, optimizedSpeed);
		SmartDashboard.putNumber("DP - " + moduleName, drivePower);
		SmartDashboard.putNumber("SR - " + moduleName, speedRegulation.getOutput());
		SmartDashboard.putNumber("GR - " + moduleName, accumulatedGR);
		*/
	}
	
	public void control(double driveSpeed, double wheelAngle) {
		SmartDashboard.putNumber("DriveSpeed-" + moduleName, getRate());
		angleGoal = convertToWheelRelative(wheelAngle);
		currentAngle = turnEncoder.getAngle();
		double wheelTurnAngle0to360 = ((angleGoal - currentAngle) + 720) % 360;
		double optimizedWheelTurnAngle; //will be set to a value between -90 and 90
	

		if (wheelTurnAngle0to360 < 5 || wheelTurnAngle0to360 > 355) {
			// stop steering
			steer.set(0);
			setDriveSpeed(driveSpeed);
		} else if (wheelTurnAngle0to360 > 175 && wheelTurnAngle0to360 < 185){
			//stop steering
			steer.set(0);
			setDriveSpeed(-driveSpeed);
		} else {
			if (wheelTurnAngle0to360 > 90 && wheelTurnAngle0to360 < 270) // for quadrants 2 & 3
			{
				optimizedWheelTurnAngle = (wheelTurnAngle0to360 - 180); // converting angles from quadrant 2 to quad 4 and converting from quad 3 to quad 1
				steer.set(optimizedWheelTurnAngle/90);
				setDriveSpeed(-driveSpeed);// go backwards
			} else // quads 1 & 4
			{
				if (wheelTurnAngle0to360 >= 270) // quad 4
				{
					optimizedWheelTurnAngle = (wheelTurnAngle0to360 - 360); // converting from large + to small -
				} else {
					optimizedWheelTurnAngle = wheelTurnAngle0to360; // quad 1, no change
				}
				steer.set(optimizedWheelTurnAngle/90);
				setDriveSpeed(driveSpeed);// forward
			}
		}
	}
}
