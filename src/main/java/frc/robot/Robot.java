/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Robot extends TimedRobot {
  private Joystick leftStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
  private Joystick rightStick = new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
  private Joystick manipulatorStick = new Joystick(RobotMap.JOYSTICK_MANIPULATOR);
  private JoystickButton hatchServoUpButton = new JoystickButton(manipulatorStick, RobotMap.JOYSTICK_BUTTON_UP);
  private JoystickButton hatchServoLevelButton = new JoystickButton(manipulatorStick, RobotMap.JOYSTICK_BUTTON_LEVEL);
  private JoystickButton hatchServoReleaseButton = new JoystickButton(manipulatorStick, RobotMap.JOYSTICK_BUTTON_RELEASE);
  private JoystickButton hatchServoBelowButton = new JoystickButton(manipulatorStick, RobotMap.JOYSTICK_BUTTON_BELOW);

  private Servo hatchServo = new Servo(RobotMap.SERVO_HATCH);

  Prototype_PWM hatchPrototype;

  TankDrive drive = new TankDrive();

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
	  hatchServo.set(0);
  }

  @Override
  public void teleopPeriodic() {
    double leftSpeed = -leftStick.getY();
    double rightSpeed = -rightStick.getY();

    drive.tankDrive(leftSpeed, rightSpeed);

	if(hatchServoUpButton.get()) {
		hatchServo.setAngle(RobotMap.SERVO_HATCH_UP_ANGLE); //90 degree angle
	}
	if(hatchServoLevelButton.get()) {
		hatchServo.setAngle(RobotMap.SERVO_HATCH_LEVEL_ANGLE); //level
	}
	if(hatchServoReleaseButton.get()) {
		hatchServo.setAngle(RobotMap.SERVO_HATCH_RELEASE_ANGLE); //slightly above level to release hatch
	}
	if(hatchServoBelowButton.get()) {
		hatchServo.setAngle(RobotMap.SERVO_HATCH_BELOW_ANGLE); // slightly below level
	} 
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}