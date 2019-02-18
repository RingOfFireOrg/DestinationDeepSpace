package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamepadSwerve extends SwerveDrive {

    public GamepadSwerve(AHRS ahrs) {
        super(ahrs);
    }

    public void runSwerve(XboxController controller, JoystickButton gyroReset, JoystickButton tuningModeActivation) {
		//this MUST be the first step so automation overrides everything else
		
		XboxController driveController = controller;
		JoystickButton gyroResetButton = gyroReset;
		JoystickButton tuningActivation = tuningModeActivation;

		int driveMode = 0;

		double leftX = squareWithSignReturn(driveController.getRawAxis(RobotMap.LEFT_STICK_X_AXIS));
		double leftY = squareWithSignReturn(-driveController.getRawAxis(RobotMap.LEFT_STICK_Y_AXIS));
		double rightX = squareWithSignReturn(driveController.getRawAxis(RobotMap.RIGHT_STICK_X_AXIS));
		double rightY = squareWithSignReturn(-driveController.getRawAxis(RobotMap.RIGHT_STICK_Y_AXIS));
		double pov = driveController.getPOV();
		double twist = squareWithSignReturn(driveController.getRawAxis(RobotMap.RIGHT_TRIGGER_AXIS) - driveController.getRawAxis(RobotMap.LEFT_TRIGGER_AXIS));

		if(controller.getYButton()){
			setRobotFrontToCargo();
		} else if(controller.getBButton()){
			setRobotFrontToHatch();
		}
		if (tuningActivation.get() == true) {
			driveMode = 1;
		} else {
			driveMode = 0;
		} 

		switch (driveMode) {
			case 0:
				//the 0s are temporary replacements for the robot relative joysticks. remember to find the opposite of the y value
				translateAndRotate(leftX, leftY, twist, ahrs.getAngle() - ahrsOffset, pov, rightX, rightY);
				break;

			case 1:
				tuningMode();
				break;

			case 2:

				break;
		
			default:
				break;
		}
		
		if (gyroResetButton.get() == true) {
			super.ahrsOffset = ahrs.getAngle();
			super.driveStraight = false;
			super.pidDrivingStraight.reset();
		}
			  
        SmartDashboard.putNumber("ahrs angle", ahrs.getAngle() - ahrsOffset);
        SmartDashboard.putNumber("POV", driveController.getPOV());
		
	}
}