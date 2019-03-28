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
	static final double MAX_STEER_POWER = 0.8;
	public boolean driveEncoderWorking = true;
	

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
		speedRegulation.setOutputRange(-RobotMap.MAX_DRIVE_POWER, RobotMap.MAX_DRIVE_POWER);
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
		drivePower *= powerInversion;
		if (drivePower > RobotMap.MAX_DRIVE_POWER) {
			drivePower = RobotMap.MAX_DRIVE_POWER;
		} else if (drivePower < -RobotMap.MAX_DRIVE_POWER) {
			drivePower = -RobotMap.MAX_DRIVE_POWER;
		}
		if (driveEncoderWorking) {
			speedRegulation.setError(drivePower - ((getRate() * RobotMap.DPS_TO_RPM) / (RobotMap.MAX_SWERVE_SPEED_IN_RPM * RobotMap.DRIVE_GEARING_RATIO)) );
			speedRegulation.update();
			optimizedSpeed = GeometricMath.limitRange(drivePower + speedRegulation.getOutput(), -RobotMap.MAX_DRIVE_POWER, RobotMap.MAX_DRIVE_POWER);
			drive.set(ControlMode.PercentOutput, optimizedSpeed);
		} else {
			drive.set(ControlMode.PercentOutput, drivePower);
		}
		
		
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
	

		if (wheelTurnAngle0to360 < 5 || wheelTurnAngle0to360 > 355) {
			// stop steering
			steer.set(ControlMode.PercentOutput, 0);
			setDriveSpeed(driveSpeed);
		} else if (wheelTurnAngle0to360 > 175 && wheelTurnAngle0to360 < 185){
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
