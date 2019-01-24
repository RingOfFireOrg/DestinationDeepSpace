package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lifter {

	// Encoder.
	private Encoder encoder = new Encoder(RobotMap.LIFT_ENCODER_A, RobotMap.LIFT_ENCODER_B, false,Encoder.EncodingType.k1X);
	private double totalRotations = 0;

	private DigitalInput upperLimitSwitch = new DigitalInput(RobotMap.INPUT_UPPER_LIMIT_SW);
	private DigitalInput lowerLimitSwitch = new DigitalInput(RobotMap.INPUT_LOWER_LIMIT_SW);

	private TalonSRX controller1 = new TalonSRX(RobotMap.CAN_LIFTER_1);
	private TalonSRX controller2 = new TalonSRX(RobotMap.CAN_LIFTER_2);

	Lifter() {
		encoder.reset();
		encoder.setSamplesToAverage(5); // noise reduction?
		encoder.setDistancePerPulse(1.0/360); // should see 1 pulse per rotation
	}

	private void debug() {
		SmartDashboard.putNumber("lift Encoder", encoder.get());
		SmartDashboard.putNumber("lift rotations", encoder.getDistance());
		SmartDashboard.putNumber("top", totalRotations);
		
		SmartDashboard.putBoolean("upper sw", upperLimitSwitch.get());
		SmartDashboard.putBoolean("lower sw", lowerLimitSwitch.get());
	}

	public void up() {
        debug();
    /*
		if (upperLimitSwitch.get()) {
			stop();
			totalRotations = encoder.getDistance();
			return;
		}
*/
		double speed = getSpeed(true);
		controller1.set(ControlMode.PercentOutput, speed);
		controller2.set(ControlMode.PercentOutput, speed);
	}
	
	private double getSpeed(boolean goingUp) {
		double speed = RobotMap.DEFAULT_LIFT_SPEED;
		if(goingUp) {
			if(totalRotations == 0) {
				speed = RobotMap.DEFAULT_FIND_SPEED;
			}
			else {
			 speed = (encoder.getDistance() * ((RobotMap.MIN_LIFT_SPEED - RobotMap.MAX_LIFT_SPEED) / totalRotations))+RobotMap.MAX_LIFT_SPEED;
			}
		}
		else {
			speed = (encoder.getDistance() * (RobotMap.MAX_LIFT_SPEED - RobotMap.MIN_LIFT_SPEED));
		}
		if(speed < RobotMap.MIN_LIFT_SPEED) {
			speed = RobotMap.MIN_LIFT_SPEED;
		}
		if(!goingUp) {
			speed = speed * -1;
		}
		return speed;
	}

	public void down() {
		debug();

		if (lowerLimitSwitch.get()) {
			encoder.reset();
			stop();
			return;
		}
		double speed = getSpeed(false);
		controller1.set(ControlMode.PercentOutput, speed);
		controller2.set(ControlMode.PercentOutput, speed);
	}

	public void stop() {
		debug();
		controller1.set(ControlMode.PercentOutput, 0);
		controller2.set(ControlMode.PercentOutput, 0);
	}
	
	public void findTop() {
		if (upperLimitSwitch.get()) {
			stop();
			totalRotations = encoder.getDistance();
		}
		controller1.set(ControlMode.PercentOutput, RobotMap.DEFAULT_FIND_SPEED);
		controller2.set(ControlMode.PercentOutput, RobotMap.DEFAULT_FIND_SPEED);		
	}
}