package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;



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

	boolean driveMode = false;
	
	SwerveDrive swerveDrive = new SwerveDrive();

  boolean alignState = false;
  AHRS ahrs;

	@Override
	public void robotInit() {

    SmartDashboard.putData("Auto choices", chooser);
    
		try {
      ahrs = new AHRS(SerialPort.Port.kUSB1);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		ahrs.reset();
	}

	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

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

	@Override
	public void teleopPeriodic() {
		double speed = Math.pow(leftStick.getMagnitude(), 2);
		double leftDirection = leftStick.getDirectionDegrees() * -1;
		double leftX = leftStick.getX();
		double leftY = leftStick.getY();
		double rightDirection = rightStick.getDirectionDegrees() * -1;
		double rightMagnitude = rightStick.getMagnitude();
		double twist = rightStick.getTwist();
		if (rightMagnitude > 0.05) {
      swerveDrive.translateAndRotate(leftX, leftY, leftDirection, ahrs.getAngle(), rightDirection, rightMagnitude);
		} else {
			if(twist < 0) {
				twist = -Math.pow(twist, 2);
			} else {
				twist = Math.pow(twist, 2);
			}
			swerveDrive.syncroDrive(speed, leftDirection, twist);
		}
		
		SmartDashboard.putNumber("Joystick output", leftDirection);
		SmartDashboard.putNumber("Joystick output speed", speed);
	
	
			
//		SmartDashboard.putNumber("Gyro output: ", ahrs.getAngle());
//				swerveDrive.syncroDrive(speed, direction, twist);	
		
	}

	@Override
	public void testPeriodic() {
		swerveDrive.individualModuleControl(frButton.get(), flButton.get(), brButton.get(), blButton.get());
		
	}

	//Code below here is not particular to swerve, temporary presence, for line alignment, auto-intervention
	public void autoAlign() {
		
		swerveDrive.syncroDrive(0.5, 90, 0);
	}

}

