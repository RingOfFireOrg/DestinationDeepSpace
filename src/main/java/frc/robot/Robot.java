package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	  
    Beak beak = Beak.getInstance();
	CargoManipulator cargoManipulator = CargoManipulator.getInstance();

	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	public XboxController driverGamepad =  new XboxController(RobotMap.DRIVER_GAMEPAD);
	public XboxController manipulatorGamepad = new XboxController(RobotMap.MANIPULATOR_GAMEPAD);

	public JoystickButton driverGamepadStartButton = new JoystickButton(driverGamepad, RobotMap.START_BUTTON_VALUE);
	public JoystickButton driverGamepadBackButton = new JoystickButton(driverGamepad, RobotMap.BACK_BUTTON_VALUE);

	public JoystickButton manipulatorAButton = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_A_BUTTON_VALUE);
	public JoystickButton manipulatorBButton = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_B_BUTTON_VALUE);
	public JoystickButton manipulatorXButton = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_X_BUTTON_VALUE);
	public JoystickButton manipulatorYButton = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_Y_BUTTON_VALUE);
	public JoystickButton manipulatorLeftBumper = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_LEFT_BUMPER_BUTTON_VALUE);
	public JoystickButton manipulatorRightBumber = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_RIGHT_BUMPER_BUTTON_VALUE);
	
	AHRS ahrs;

	boolean driveMode = false;

	boolean autoClimbMode = false;

//	Vision limelight = new Vision();
	
	GamepadSwerve swerveDrive;

	//ManipulatorStation manipulatorStation = new ManipulatorStation();

	ClimberController climberController;

	RobotTest robotTest = new RobotTest();


	@Override
	public void robotInit() {
		ahrs = new AHRS(SerialPort.Port.kUSB);
		ahrs.reset();
	
		swerveDrive = new GamepadSwerve(ahrs);
		climberController = new ClimberController(swerveDrive);
	}

	@Override
	public void teleopPeriodic() {
		//if (/*limelight.isAutomationRunning() || autoClimbMode*/ false) {

	//	} else {
			swerveDrive.runSwerve(driverGamepad, driverGamepadStartButton, driverGamepadBackButton);
			beakControl();
			cargoManipulatorControl();
			climberController.run();
	//	}
	}

	@Override
	public void testPeriodic() {
		//robotTest.runTest();
		cargoManipulator.currentAngle();
	}

	public void beakControl() {
		if (manipulatorAButton.get() == true) {
			beak.open();
		} else if (manipulatorBButton.get() == true) {
			beak.close();
		}
	}

	public void cargoManipulatorControl() {
		if (manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER_AXIS) > 0.3) {
			cargoManipulator.setOut();
		} else if (manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER_AXIS) > 0.3) {
			cargoManipulator.setIn();
		} else if (manipulatorGamepad.getPOV() == 0) {
			cargoManipulator.overrideTarget(-1);
		} else if (manipulatorGamepad.getPOV() == 180) {
			cargoManipulator.overrideTarget(1);
		}
			
			else {
			cargoManipulator.setOff();
		}
		if (manipulatorLeftBumper.get() == true) {
			cargoManipulator.setUp();
		} else if (manipulatorRightBumber.get() == true) {
			cargoManipulator.setIntake();
		} else if (manipulatorXButton.get() == true) {
			cargoManipulator.setCargoShip();
		} else if (manipulatorYButton.get() == true) {
			cargoManipulator.setLowerRocket();
		} else {
			cargoManipulator.setStall();
		}
		cargoManipulator.updateCargo();
	}

}

