package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TankDrive extends DifferentialDrive {
	
	private static final int ROTATIONS_PER_INCH = 5;
	private Encoder leftEncoder = new Encoder(RobotMap.DRIVE_TRAIN_LEFT_ENCODER_A, RobotMap.DRIVE_TRAIN_LEFT_ENCODER_B, false, Encoder.EncodingType.k1X);
	private Encoder rightEncoder = new Encoder(RobotMap.DRIVE_TRAIN_RIGHT_ENCODER_A, RobotMap.DRIVE_TRAIN_RIGHT_ENCODER_B, false, Encoder.EncodingType.k1X);
	TankDrive(Victor left, Victor right) {
		super(left, right);
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
