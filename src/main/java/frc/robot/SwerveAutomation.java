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

	// public Point translationVector() {
	// 	//returns a point of the translation vector in a robot relative
	// 	if (swerveDrive.frontRightCMPerSecond() != 0 && swerveDrive.backLeftCMPerSecond() != 0) { //the != should be to what the encoders return if they are not working
	// 		Point frWheelOriginalVector = new Point(RobotMap.ROBOT_STANCE_X_IN_CM / 2, RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
	// 		Point blWheelOriginalVector = new Point(-RobotMap.ROBOT_STANCE_X_IN_CM / 2, -RobotMap.ROBOT_STANCE_Y_IN_CM / 2);
	// 		//Point fr
	// 	}
	// }

}