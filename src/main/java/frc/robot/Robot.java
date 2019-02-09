package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Climber;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	Climber climberFront;
  	Climber climberBack;
 	Climber climberLeftWheel;
  	Climber climberRightWheel;

	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	private Joystick manipulatorStickL = new Joystick(2);
	private Joystick manipulatorStickR = new Joystick(3);
	SendableChooser<String> chooser = new SendableChooser<>();
	JoystickButton frButton = new JoystickButton(leftStick, 6);
	JoystickButton flButton = new JoystickButton(leftStick, 5);
	JoystickButton brButton = new JoystickButton(leftStick, 4);
	JoystickButton blButton = new JoystickButton(leftStick, 3);
	JoystickButton trigger = new JoystickButton(leftStick, 1);
	JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
	JoystickButton tuningActivation = new JoystickButton(leftStick, 7);
	JoystickButton stickTriggerL = new JoystickButton(manipulatorStickL, 1);
	JoystickButton stickTriggerR = new JoystickButton(manipulatorStickR, 1);
	JoystickButton stickThumbL = new JoystickButton(manipulatorStickL, 2);
	JoystickButton stickThumbR = new JoystickButton(manipulatorStickR, 2);

	boolean driveMode = false;
	
	SwerveDrive swerveDrive = new SwerveDrive();

  	boolean alignState = false;

	@Override
	public void robotInit() {
		SwerveDrive.swerveInit();

		climberFront = new Climber(RobotMap.CAN_CLIMBER_FRONT, RobotMap.SPEED_DEFAULT_TEST);
		climberBack = new Climber(RobotMap.CAN_CLIMBER_BACK, RobotMap.SPEED_DEFAULT_TEST);
		climberLeftWheel = new Climber(RobotMap.CAN_CLIMBER_WHEEL_LEFT, RobotMap.SPEED_DEFAULT_TEST);
		climberRightWheel = new Climber(RobotMap.CAN_CLIMBER_WHEEL_RIGHT, RobotMap.SPEED_DEFAULT_TEST);
	}

	@Override
	public void teleopPeriodic() {
		SwerveDrive.runSwerve(leftStick, rightStick, rightTrigger, tuningActivation);

		double yPosL = manipulatorStickL.getY();
    	double yPosR = manipulatorStickR.getY();
    	boolean stickTriggerLeft = stickTriggerL.get();
    	boolean stickTriggerRight = stickTriggerR.get();
    	boolean stickThumbLeft = stickThumbL.get();
		boolean stickThumbRight = stickThumbR.get();
		
		// The 0.25 and -0.25 are so that the joystick doesn't have to be perfectly centered to stop
    	if (yPosL < 0.25) {
			climberLeftWheel.forward();
	 	} else if (yPosL > -0.25) {
			climberLeftWheel.reverse();
	  	} else {
			climberLeftWheel.stop();
	  	}
  
	  	if (yPosR < 0.25) {
			climberRightWheel.forward();
	  	} else if (yPosR > -0.25) {
			climberRightWheel.reverse();
	  	} else {
			climberRightWheel.stop();
	  	}
  
	  	if (stickTriggerLeft) {
			climberFront.reverse();
	  	} else if (stickThumbLeft) {
			climberFront.forward();
	  	} else {
			climberFront.stop();
	  	}
  
	  	if (stickTriggerRight) {
			climberBack.reverse();
	  	} else if (stickThumbRight) {
			climberBack.forward();
	  	} else {
			climberBack.stop();
	  	}
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

