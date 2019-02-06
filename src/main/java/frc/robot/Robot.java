/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Prototype_CAN;

/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends TimedRobot {
  Prototype_CAN climberFront;
  Prototype_CAN climberBack;
  Prototype_CAN climberWheelLeft;
  Prototype_CAN climberWheelRight;

  private Joystick leftStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
  private Joystick rightStick = new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
  private Joystick manipulatorStickL = new Joystick(RobotMap.JOYSTICK_MANIPULATORL);
  private Joystick manipulatorStickR = new Joystick(RobotMap.JOYSTICK_MANIPULATORR);
  public JoystickButton stickTriggerL = new JoystickButton(manipulatorStickL, 0);
  public JoystickButton stickTriggerR = new JoystickButton(manipulatorStickR, 0);
  public JoystickButton stickThumbL = new JoystickButton(manipulatorStickL, 1);
  public JoystickButton stickThumbR = new JoystickButton(manipulatorStickR, 1);

  

  TankDrive drive = new TankDrive();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    climberFront = new Prototype_CAN(RobotMap.CAN_TEST_CLIMBER_FRONT, RobotMap.SPEED_DEFAULT_TEST);
    climberBack = new Prototype_CAN(RobotMap.CAN_TEST_CLIMBER_BACK, RobotMap.SPEED_DEFAULT_TEST);
    climberWheelLeft = new Prototype_CAN(RobotMap.CAN_CLIMBER_WHEEL_LEFT, RobotMap.SPEED_DEFAULT_TEST);
    climberWheelRight = new Prototype_CAN(RobotMap.CAN_CLIMBER_WHEEL_RIGHT, RobotMap.SPEED_DEFAULT_TEST);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once we go into autonomous mode
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous. (approx 20ms)
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called when you switch into teleop mode on the driver
   * station.
   */
  @Override
  public void teleopInit() {

  }

  /**
   * This function is called periodically during operator control. (approx 20ms)
   */
  @Override
  public void teleopPeriodic() {
    double leftSpeed = -leftStick.getY();
    double rightSpeed = -rightStick.getY();
    double yPosL = manipulatorStickL.getY();
    double yPosR = manipulatorStickR.getY();
    boolean stickTriggerLeft = stickTriggerL.get();
    boolean stickTriggerRight = stickTriggerR.get();
    boolean stickThumbLeft = stickThumbL.get();
    boolean stickThumbRight = stickThumbR.get();


    drive.tankDrive(leftSpeed, rightSpeed);


    // The 0.25 and -0.25 are so that the joystick doesn't have to be perfectly
    // centered to stop
    if (yPosL > 0.25) {
      climberWheelLeft.forward();
    } else if (yPosL < -0.25) {
      climberWheelLeft.reverse();
    } else {
      climberWheelLeft.stop();
    }

    if (yPosR > 0.25) {
      climberWheelRight.forward();
    } else if (yPosR < -0.25) {
      climberWheelRight.reverse();
    } else {
      climberWheelRight.stop();
    }

    if (stickTriggerLeft = true) {
      climberFront.forward();
    } else {
      climberFront.stop();
    }

    if (stickThumbLeft = true) {
      climberFront.reverse();
    }  else {
      climberFront.stop();
    }

    if (stickTriggerRight = true) {
      climberBack.forward();
    }  else {
      climberBack.stop();
    }

    if (stickThumbRight = true) {
      climberBack.reverse();
    }  else {
      climberBack.stop();
    }

  }

  /**
   * This function is called when you switch into teleop mode on the driver
   * station.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode. (approx 20ms)
   */
  @Override
  public void testPeriodic() {
  }
}
