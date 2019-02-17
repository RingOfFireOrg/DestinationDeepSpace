package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class RobotTest {

    public static final double TEST_DRIVE_SPEED = 0.5;
    public static final double TEST_STEER_SPEED = 0.5;
    public XboxController driverGamepad = new XboxController(RobotMap.DRIVER_GAMEPAD);
    public JoystickButton frontLeftTestButton = new JoystickButton(driverGamepad, RobotMap.TEST_FRONT_LEFT_BUTTON);
    public JoystickButton frontRightTestButton = new JoystickButton(driverGamepad, RobotMap.TEST_FRONT_RIGHT_BUTTON);
    public JoystickButton backLeftTestButton = new JoystickButton(driverGamepad, RobotMap.TEST_BACK_LEFT_BUTTON);
    public JoystickButton backRightTestButton = new JoystickButton(driverGamepad, RobotMap.TEST_BACK_RIGHT_BUTTON);

    private SwerveDrive swerveDrive;

    public void initTest(SwerveDrive inputDrive) {
        this.swerveDrive = inputDrive;
    }

    public void runTest() {
        if (frontLeftTestButton.get()) {
            swerveDrive.testSwerveModule(true, true, TEST_DRIVE_SPEED, TEST_STEER_SPEED);
        } else {
            swerveDrive.testSwerveModule(true, true, 0, 0);
        }

        if (frontRightTestButton.get()) {
            swerveDrive.testSwerveModule(true, false, TEST_DRIVE_SPEED, TEST_STEER_SPEED);
        } else {
            swerveDrive.testSwerveModule(true, false, 0, 0);
        }
        if (backLeftTestButton.get()) {
            swerveDrive.testSwerveModule(false, true, TEST_DRIVE_SPEED, TEST_STEER_SPEED);
        } else {
            swerveDrive.testSwerveModule(false, true, 0, 0);
        }
        if (backRightTestButton.get()) {
            swerveDrive.testSwerveModule(false, false, TEST_DRIVE_SPEED, TEST_STEER_SPEED);
        } else {
            swerveDrive.testSwerveModule(false, false, 0, 0);
        }
    }

}