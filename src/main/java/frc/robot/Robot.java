package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	SendableChooser<String> chooser = new SendableChooser<>();
	JoystickButton frButton = new JoystickButton(leftStick, 6);
	JoystickButton flButton = new JoystickButton(leftStick, 5);
	JoystickButton brButton = new JoystickButton(leftStick, 4);
	JoystickButton blButton = new JoystickButton(leftStick, 3);
	JoystickButton trigger = new JoystickButton(leftStick, 1);
	JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
	JoystickButton tuningActivation = new JoystickButton(leftStick, 7);

	boolean driveMode = false;
	
	SwerveDrive swerveDrive = new SwerveDrive();

  boolean alignState = false;
  
 

	@Override
	public void robotInit() {
		SwerveDrive.swerveInit();
	}

	@Override
	public void teleopPeriodic() {
		SwerveDrive.runSwerve(leftStick, rightStick, rightTrigger, tuningActivation);
	}

	@Override
	public void testPeriodic() {
		//swerveDrive.individualModuleControl(frButton.get(), flButton.get(), brButton.get(), blButton.get());
		
	}

	//Code below here is not particular to swerve, temporary presence, for line alignment, auto-intervention
	public void autoAlign() {
		
		//swerveDrive.syncroDrive(0.5, 90, 0);
	}

}

