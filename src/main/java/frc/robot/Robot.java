package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	Joystick commandStick = new Joystick(0);
	SendableChooser<String> chooser = new SendableChooser<>();
	JoystickButton frButton = new JoystickButton(commandStick, 6);
	JoystickButton flButton = new JoystickButton(commandStick, 5);
	JoystickButton brButton = new JoystickButton(commandStick, 4);
	JoystickButton blButton = new JoystickButton(commandStick, 3);
	JoystickButton trigger = new JoystickButton(commandStickk, 1);
	
	SwerveDrive swerveDrive = new SwerveDrive();

	boolean alignState = false;
//	AHRS ahrs;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		try {
//			ahrs = new AHRS(SerialPort.Port.kUSB1);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

//		ahrs.reset();
	}

	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		double twist;
		double speed = Math.pow(commandStick.getMagnitude(), 2);
		double direction = commandStick.getDirectionDegrees() * -1;
		twist = commandStick.getTwist();
		if(twist < 0) {
			twist = -Math.pow(twist, 2);
		} else {
			twist = Math.pow(twist, 2);
		}
		SmartDashboard.putNumber("Joystick output", direction);
		SmartDashboard.putNumber("Joystick output speed", speed);
	
		if (trigger.get()) {
			autoAlign();
		}	else {
			swerveDrive.synchroDrive(speed, direction, twist);
			alignState = false;
		}
//		SmartDashboard.putNumber("Gyro output: ", ahrs.getAngle());
//				swerveDrive.syncroDrive(speed, direction, twist);	
		
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		swerveDrive.individualModuleControl(frButton.get(), flButton.get(), brButton.get(), blButton.get());
		
	}

	//Code below here is not particular to swerve, temporary presence, for line alignment, auto-intervention

	public void autoAlign() {
		
		swerveDrive.synchroDrive(0.5, 90, 0);
	}

}

