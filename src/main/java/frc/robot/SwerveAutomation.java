package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class SwerveAutomation {

	private static SwerveAutomation swerveAutomation;

	SwerveAutomation(AHRS ahrs, SwerveDrive swerveDrive) {

	}

	public static SwerveAutomation getInstance(AHRS ahrs, SwerveDrive swerveDrive) {
		if (swerveAutomation == null) {
			swerveAutomation = new SwerveAutomation(ahrs, swerveDrive);
		}
		return swerveAutomation;
	}

	public static Point translationVector() {
		if (swerveDrive.frontRightCMPerSecond() != null && swerveDrive.backLeftCMPerSecond != null) {
			
		}
	}
}