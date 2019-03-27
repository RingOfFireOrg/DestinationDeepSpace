package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class SwerveAutomation {

	private static SwerveAutomation swerveAutomation;

	private static SwerveDrive swerveDrive;

	private static AHRS ahrs;

	SwerveAutomation(AHRS ahrs, SwerveDrive swerveDrive) {
		this.swerveDrive = swerveDrive;
		this.ahrs = ahrs;
	}

	public static SwerveAutomation getInstance(AHRS ahrs, SwerveDrive swerveDrive) {
		if (swerveAutomation == null) {
			swerveAutomation = new SwerveAutomation(ahrs, swerveDrive);
		}
		return swerveAutomation;
	}

	public Point translationVector() {
		//returns a point of the translation vector in a robot relative
		if (swerveDrive.frontRightCMPerSecond() != 0 && swerveDrive.frontLeftCMPerSecond() != 0 && swerveDrive.backLeftCMPerSecond() != 0 && swerveDrive.backRightCMPerSecond() != 0) { //the != should be to what the encoders return if they are not working
			Point frWheelOriginalVector = new Point(RobotMap.ROBOT_STANCE_X_IN_CM / 2, RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
			Point flWheelOriginalVector = new Point(-RobotMap.ROBOT_STANCE_X_IN_CM / 2, RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
			Point blWheelOriginalVector = new Point(-RobotMap.ROBOT_STANCE_X_IN_CM / 2, -RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
			Point brWheelOriginalVector = new Point(RobotMap.ROBOT_STANCE_X_IN_CM / 2, -RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
			
			Point frWheelTransformationVector = GeometricMath.rotateVector(new Point(0, swerveDrive.frontRightCMPerSecond()), swerveDrive.frontRightAngle());
			Point flWheelTransformationVector = GeometricMath.rotateVector(new Point(0, swerveDrive.frontLeftCMPerSecond()), swerveDrive.frontLeftAngle());
			Point blWheelTransformationVector = GeometricMath.rotateVector(new Point(0, swerveDrive.backLeftCMPerSecond()), swerveDrive.backLeftAngle());
			Point brWheelTransformationVector = GeometricMath.rotateVector(new Point(0, swerveDrive.backRightCMPerSecond()), swerveDrive.backRightAngle());

			Point frWheelTransformedVector = GeometricMath.vectorAddition(frWheelOriginalVector, frWheelOriginalVector);
			Point flWheelTransformedVector = GeometricMath.vectorAddition(flWheelOriginalVector, flWheelOriginalVector);
			Point blWheelTransformedVector = GeometricMath.vectorAddition(blWheelOriginalVector, blWheelOriginalVector);
			Point brWheelTransformedVector = GeometricMath.vectorAddition(brWheelOriginalVector, brWheelOriginalVector);

			Point originalCenter = GeometricMath.midPoint(GeometricMath.midPoint(frWheelOriginalVector, blWheelOriginalVector), GeometricMath.midPoint(flWheelOriginalVector, brWheelOriginalVector));
			Point translatedCenter = GeometricMath.midPoint(GeometricMath.midPoint(frWheelTransformedVector, blWheelTransformedVector), GeometricMath.midPoint(flWheelTransformedVector, brWheelTransformedVector));
		} else {

		}

	}

}