package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PIDController;

//need to ifx
public class SwerveModule {
	TalonSRX drive;
	VictorSPX steer;
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
	int powerInversion = 1;
	static final double MAX_DRIVE_POWER = 0.8;
	static final double MAX_STEER_POWER = 0.8;
	

	//will need to make changes to the input --Encoder driveRotEncoder <-- add to constructor
	public SwerveModule(TalonSRX driveMotor, VictorSPX steerMotor, AbsoluteAnalogEncoder steerEncoder, double zeroValue, Encoder driveRotEncoder, String name) {
		this.zeroValue = zeroValue;
		drive = driveMotor;
		steer = steerMotor;
		turnEncoder = steerEncoder;
		moduleName = name;
		driveEncoder = driveRotEncoder;

		driveEncoder.reset();
		driveEncoder.setDistancePerPulse(18); //in degrees (360)/(20 pulses per rotation)

		speedRegulation = new PID(0, -0.000001, 0);
		speedRegulation.setOutputRange(-1, 1);
		speedRegulation.reset();
	}

	public void invertModule() {
		powerInversion = -1;
	}

	public void resetModule() {
		speedRegulation.reset();
	}

	public double convertToWheelRelative(double wheelAngleGoal) {
		return ((wheelAngleGoal + zeroValue) + 720) % 360;
	}

	public double convertToRobotRelative(double wheelAngleGoal) {
		return ((wheelAngleGoal - zeroValue) + 720) % 360;
	}

	public double robotRelativeAngle() {
		return convertToRobotRelative(getAngle());
	}

	public double getAngle() {
		return turnEncoder.getAngle();
	}

	public double getRate() {
		return driveEncoder.getRate(); //in degrees per second
	}

	public void stop() {
		drive.set(ControlMode.PercentOutput, 0);
		steer.set(ControlMode.PercentOutput, 0);
	}

	public void setDriveSpeed(double drivePower) {
		/*if (moduleName == "BackLeft") {
			accumulatedGR += drivePower;
			speedRegulation.setError((drivePower * 28000) - getRate());
			speedRegulation.update();
			optimizedSpeed = drivePower + speedRegulation.getOutput();
			if (optimizedSpeed > MAX_DRIVE_POWER) optimizedSpeed = MAX_DRIVE_POWER;
			if (optimizedSpeed < -MAX_DRIVE_POWER) optimizedSpeed = -MAX_DRIVE_POWER;
			drive.set(optimizedSpeed);
			//SmartDashboard.putNumber("OS - " + moduleName, optimizedSpeed);
			//SmartDashboard("encoder a", driveEncoder.());
		} else { */ 
			if (drivePower > MAX_DRIVE_POWER) {
				drivePower = MAX_DRIVE_POWER;
			} else if (drivePower < -MAX_DRIVE_POWER) {
				drivePower = -MAX_DRIVE_POWER;
			}
			drive.set(ControlMode.PercentOutput, powerInversion * drivePower);
		//} 
		//}
		
		//SmartDashboard.putNumber("OS - " + moduleName, optimizedSpeed);
		//SmartDashboard.putNumber("DP - " + moduleName, drivePower);
		//SmartDashboard.putNumber("SR - " + moduleName, speedRegulation.getOutput());
		//SmartDashboard.putNumber("GR - " + moduleName, accumulatedGR);
		
	}

	public void setSteerSpeed(double steerPower) {

		if (steerPower > MAX_STEER_POWER) {
			steerPower = MAX_STEER_POWER;
		} else if (steerPower < -MAX_STEER_POWER) {
			steerPower = -MAX_STEER_POWER;
		}
		steer.set(ControlMode.PercentOutput, steerPower);
		
	}
	
	public void control(double driveSpeed, double wheelAngle) {
		SmartDashboard.putNumber("WheelSpeed:D/S-" + moduleName, getRate());
		angleGoal = convertToWheelRelative(wheelAngle);
		currentAngle = turnEncoder.getAngle();
		double wheelTurnAngle0to360 = ((angleGoal - currentAngle) + 720) % 360;
		double optimizedWheelTurnAngle; //will be set to a value between -90 and 90
	

		if (wheelTurnAngle0to360 < 2 || wheelTurnAngle0to360 > 358) {
			// stop steering
			steer.set(ControlMode.PercentOutput, 0);
			setDriveSpeed(driveSpeed);
		} else if (wheelTurnAngle0to360 > 178 && wheelTurnAngle0to360 < 188){
			//stop steering
			steer.set(ControlMode.PercentOutput, 0);
			setDriveSpeed(-driveSpeed);
		} else {
			if (wheelTurnAngle0to360 > 90 && wheelTurnAngle0to360 < 270) // for quadrants 2 & 3
			{
				optimizedWheelTurnAngle = (wheelTurnAngle0to360 - 180); // converting angles from quadrant 2 to quad 4 and converting from quad 3 to quad 1
				setSteerSpeed(optimizedWheelTurnAngle/90);
				setDriveSpeed(-driveSpeed);// go backwards
			} else // quads 1 & 4
			{
				if (wheelTurnAngle0to360 >= 270) // quad 4
				{
					optimizedWheelTurnAngle = (wheelTurnAngle0to360 - 360); // converting from large + to small -
				} else {
					optimizedWheelTurnAngle = wheelTurnAngle0to360; // quad 1, no change
				}
				setSteerSpeed(optimizedWheelTurnAngle/90);
				setDriveSpeed(driveSpeed);// forward
			}
		}
	}

	public double driveEncoderDegrees() {
		return driveEncoder.get();
	}

	public double driveEncoderDegrees(double relative) {
		return driveEncoder.get() - relative;
	}
}
