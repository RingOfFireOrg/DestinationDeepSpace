package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

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

	public Joystick manipulatorPanel = new Joystick(RobotMap.MANIPULATOR_PANEL);
	public XboxController driverGamepad = new XboxController(RobotMap.DRIVER_GAMEPAD);
	public XboxController manipulatorGamepad = new XboxController(RobotMap.MANIPULATOR_GAMEPAD);

	public JoystickButton driverGamepadStartButton = new JoystickButton(driverGamepad, RobotMap.START_BUTTON_VALUE);
	public JoystickButton driverGamepadBackButton = new JoystickButton(driverGamepad, RobotMap.BACK_BUTTON_VALUE);

	public JoystickButton manipulatorAButton = new JoystickButton(manipulatorGamepad,
			RobotMap.MANIPULATOR_A_BUTTON_VALUE);
	public JoystickButton manipulatorBButton = new JoystickButton(manipulatorGamepad,
			RobotMap.MANIPULATOR_B_BUTTON_VALUE);
	public JoystickButton manipulatorXButton = new JoystickButton(manipulatorGamepad,
			RobotMap.MANIPULATOR_X_BUTTON_VALUE);
	public JoystickButton manipulatorYButton = new JoystickButton(manipulatorGamepad,
			RobotMap.MANIPULATOR_Y_BUTTON_VALUE);
	public JoystickButton manipulatorLeftBumper = new JoystickButton(manipulatorGamepad,
			RobotMap.MANIPULATOR_LEFT_BUMPER_BUTTON_VALUE);
	public JoystickButton manipulatorRightBumber = new JoystickButton(manipulatorGamepad,
			RobotMap.MANIPULATOR_RIGHT_BUMPER_BUTTON_VALUE);
	public JoystickButton manipulatorStartButton = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_START_BUTTON_VALUE);
	public JoystickButton manipulatorBackButton = new JoystickButton(manipulatorGamepad, RobotMap.MANIPULATOR_BACK_BUTTON_VALUE);
	
	public JoystickButton manipulatorPanelAutoIntakeHatch = new JoystickButton(manipulatorPanel, RobotMap.AUTO_INTAKE_HATCH_BUTTON);
	public JoystickButton manipulatorPanelAutoScoreHatch = new JoystickButton(manipulatorPanel, RobotMap.AUTO_SCORE_HATCH_BUTTON);
	public JoystickButton manipulatorPanelOpenBeak = new JoystickButton(manipulatorPanel, RobotMap.OPEN_BEAK_BUTTON);
	public JoystickButton manipulatorPanelCloseBeak = new JoystickButton(manipulatorPanel, RobotMap.CLOSE_BEAK_BUTTON);
	public JoystickButton manipulatorPanelAutoMidRocket = new JoystickButton(manipulatorPanel, RobotMap.AUTO_MID_ROCKET_CARGO_SCORE_BUTTON);
	public JoystickButton manipulatorPanelAutoLowRocket = new JoystickButton(manipulatorPanel, RobotMap.AUTO_LOW_ROCKET_CARGO_SCORE_BUTTON); 

	AHRS ahrs;

	boolean driveMode = false;

	boolean autoClimbMode = false;

	// Vision limelight = new Vision();

	GamepadSwerve swerveDrive;

	// ManipulatorStation manipulatorStation = new ManipulatorStation();

	ClimberController climberController;

	RobotTest robotTest = new RobotTest();

	@Override
	public void robotInit() {
		ahrs = new AHRS(SerialPort.Port.kUSB);
		ahrs.reset();

		swerveDrive = new GamepadSwerve(ahrs);
		climberController = new ClimberController(swerveDrive, ahrs);
		SmartDashboard.putNumber("Version #", 5);
	}

	@Override
	public void teleopPeriodic() {
		// if (/*limelight.isAutomationRunning() || autoClimbMode*/ false) {

		// } else {
		swerveDrive.runSwerve(driverGamepad, driverGamepadStartButton, driverGamepadBackButton);
		beakControl();
		cargoManipulatorControl();
		climberController.run();
		// }
	}

	@Override
	public void testPeriodic() {
		// robotTest.runTest();
		cargoManipulator.currentAngle();
		beak.close();
	}

	public void beakControl() {
		if (manipulatorPanelOpenBeak.get()) {
			beak.open();
		} else if (manipulatorPanelCloseBeak.get()) {
			beak.close();
		}
	}

	public void cargoManipulatorControl() {
		/*
		if (manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER_AXIS) > 0.3) {
			cargoManipulator.setToIntakePosition();
		} else if (manipulatorGamepad.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER_AXIS) > 0.3) {
			cargoManipulator.setToUpPosition();
		}
		*/
		// piece below is meant to make the arm go up or down unbounded
		/*
		 * else if (manipulatorGamepad.getPOV() == 0) {
		 * cargoManipulator.overrideTarget(-1); } else if (manipulatorGamepad.getPOV()
		 * == 180) { cargoManipulator.overrideTarget(1); }
		 */
		/*
		 else {
			cargoManipulator.setWheelsOff();
		}
		*/

		//eventually this needs a case for middle rocket
		if (manipulatorPanel.getRawButton(RobotMap.CARGO_ARM_UP_POSITION_BUTTON)) {
			cargoManipulator.setToUpPosition();
		} else if (manipulatorPanel.getRawButton(RobotMap.CARGO_ARM_INTAKE_POSITION_BUTTON)) {
			cargoManipulator.setToIntakePosition();
		} else if (manipulatorPanel.getRawButton(RobotMap.CARGO_ARM_CARGO_SHIP_POSITION_BUTTON)) {
			cargoManipulator.setToCargoShipPosition();
		} else if (manipulatorPanel.getRawButton(RobotMap.CARGO_ARM_LOW_ROCKET_POSITION_BUTTON)) {
			cargoManipulator.setToLowerRocketPosition();
		}else {
			cargoManipulator.setToCurrentPosition();
			// essentially keeps it steady at wherever we are so that it doesn't fall down
		}
		//Wheel control for manipulator panel
		double cargoWheelsSpeed = manipulatorPanel.getRawAxis(0); // check if this is the correct axis; also check if this is the right place to get the axis
		if (cargoWheelsSpeed > 0.2){
			cargoManipulator.setWheelsIn();
		} else if (cargoWheelsSpeed < -0.2){
			cargoManipulator.setWheelsOut();
		} else {
			cargoManipulator.setWheelsOff();
		}
	
		double cargoArmSpeed = manipulatorGamepad.getRawAxis(5);
		if(cargoArmSpeed > 0.2){
			cargoManipulator.moveArmUp(cargoArmSpeed);
		} else if (cargoArmSpeed < -0.2){
			cargoManipulator.moveArmDown(-cargoArmSpeed);
		} else {
			cargoManipulator.stopArm();
		}

		if(manipulatorXButton.get()){
			cargoManipulator.setWheelsIn();
		}else if(manipulatorYButton.get()){
			cargoManipulator.setWheelsOut();
		} else {
			cargoManipulator.setWheelsOff();
		}
	}

}
