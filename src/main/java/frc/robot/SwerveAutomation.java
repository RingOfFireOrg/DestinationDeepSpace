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

	public Point translationVectorInCM() {
		// returns a point of the translation vector in a robot relative
		Point frWheelOriginalVector = new Point(RobotMap.ROBOT_STANCE_X_IN_CM / 2, RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
		Point flWheelOriginalVector = new Point(-RobotMap.ROBOT_STANCE_X_IN_CM / 2, RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
		Point blWheelOriginalVector = new Point(-RobotMap.ROBOT_STANCE_X_IN_CM / 2, -RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
		Point brWheelOriginalVector = new Point(RobotMap.ROBOT_STANCE_X_IN_CM / 2, -RobotMap.ROBOT_STANCE_Y_IN_CM / 2);

		Point frWheelTransformationVector = GeometricMath
				.rotateVector(new Point(0, swerveDrive.frontRightCMPerSecond()), swerveDrive.frontRightAngle());
		Point flWheelTransformationVector = GeometricMath.rotateVector(new Point(0, swerveDrive.frontLeftCMPerSecond()),
				swerveDrive.frontLeftAngle());
		Point blWheelTransformationVector = GeometricMath.rotateVector(new Point(0, swerveDrive.backLeftCMPerSecond()),
				swerveDrive.backLeftAngle());
		Point brWheelTransformationVector = GeometricMath.rotateVector(new Point(0, swerveDrive.backRightCMPerSecond()),
				swerveDrive.backRightAngle());

		Point frWheelTransformedVector = GeometricMath.vectorAddition(frWheelOriginalVector, frWheelTransformationVector);
		Point flWheelTransformedVector = GeometricMath.vectorAddition(flWheelOriginalVector, flWheelTransformationVector);
		Point blWheelTransformedVector = GeometricMath.vectorAddition(blWheelOriginalVector, blWheelTransformationVector);
		Point brWheelTransformedVector = GeometricMath.vectorAddition(brWheelOriginalVector, brWheelTransformationVector);

		if (swerveDrive.frSpeedEncoderIsWorking && swerveDrive.flSpeedEncoderIsWorking
				&& swerveDrive.blSpeedEncoderIsWorking && swerveDrive.brSpeedEncoderIsWorking) {
			Point originalCenter = GeometricMath.midPoint(
					GeometricMath.midPoint(frWheelOriginalVector, blWheelOriginalVector),
					GeometricMath.midPoint(flWheelOriginalVector, brWheelOriginalVector));
			Point translatedCenter = GeometricMath.midPoint(
					GeometricMath.midPoint(frWheelTransformedVector, blWheelTransformedVector),
					GeometricMath.midPoint(flWheelTransformedVector, brWheelTransformedVector));
			return GeometricMath.vectorSubtraction(translatedCenter, originalCenter);
		} else if (swerveDrive.frSpeedEncoderIsWorking && swerveDrive.blSpeedEncoderIsWorking) {
			Point originalCenter = GeometricMath.midPoint(frWheelOriginalVector, blWheelOriginalVector);
			Point translatedCenter = GeometricMath.midPoint(frWheelTransformedVector, blWheelTransformedVector);
			return GeometricMath.vectorSubtraction(translatedCenter, originalCenter);
		} else if (swerveDrive.flSpeedEncoderIsWorking && swerveDrive.brSpeedEncoderIsWorking) {
			Point originalCenter = GeometricMath.midPoint(flWheelOriginalVector, brWheelOriginalVector);
			Point translatedCenter = GeometricMath.midPoint(flWheelTransformedVector, brWheelTransformedVector);
			return GeometricMath.vectorSubtraction(translatedCenter, originalCenter);
		} else {
			return new Point(0, 0);
		}

	}

}