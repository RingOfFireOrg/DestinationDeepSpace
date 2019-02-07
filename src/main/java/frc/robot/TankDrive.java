package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class TankDrive extends DifferentialDrive {
	
	private static final int ROTATIONS_PER_INCH = 5;
	private Encoder leftEncoder = new Encoder(RobotMap.DRIVE_TRAIN_LEFT_ENCODER_A, RobotMap.DRIVE_TRAIN_LEFT_ENCODER_B, false, Encoder.EncodingType.k1X);
	private Encoder rightEncoder = new Encoder(RobotMap.DRIVE_TRAIN_RIGHT_ENCODER_A, RobotMap.DRIVE_TRAIN_RIGHT_ENCODER_B, false, Encoder.EncodingType.k1X);
	
	TankDrive() {
		super(new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN_MOTOR_FRONT_LEFT), new WPI_VictorSPX(RobotMap.CAN_MOTOR_BACK_LEFT)),
			  new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN_MOTOR_FRONT_RIGHT), new WPI_VictorSPX(RobotMap.CAN_MOTOR_BACK_RIGHT)));
		initEncoder(leftEncoder);
		initEncoder(rightEncoder);
	}
	
	private void initEncoder(Encoder encoder) {
		encoder.reset();
		encoder.setSamplesToAverage(5); // noise reduction?
		encoder.setDistancePerPulse(1.0/360); // should see 1 pulse per rotation
	}
	
	public void printEncoderValue() {
		SmartDashboard.putNumber("left Encoder", leftEncoder.get());
		SmartDashboard.putNumber("left rotations", leftEncoder.getDistance());
		SmartDashboard.putNumber("right Encoder", rightEncoder.get());
		SmartDashboard.putNumber("right rotations", rightEncoder.getDistance());
	}

	public double getLeftDistance() {
		return leftEncoder.getDistance() * ROTATIONS_PER_INCH;
	}
	
	public double getRightDistance() {
		return rightEncoder.getDistance() * ROTATIONS_PER_INCH;
	}
	
	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}
}
