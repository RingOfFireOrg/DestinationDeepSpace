/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GamepadBase;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Prototype_CAN;
//import sun.net.www.content.text.Generic;

/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends TimedRobot {
  Prototype_CAN climberFront;
  Prototype_CAN climberBack;
  Prototype_CAN climberWheelLeft;
  Prototype_CAN climberWheelRight;

  private GenericHID controller = new Joystick(RobotMap.GAMEPAD_CONTROLLER);

  // public JoystickButton triggerL = new JoystickButton(controller, 2);
  // public JoystickButton triggerR = new JoystickButton(controller, 3);

  public JoystickButton bumperL = new JoystickButton(controller, RobotMap.LEFT_BUMPER_BUTTON);
  public JoystickButton bumperR = new JoystickButton(controller, RobotMap.RIGHT_BUMPER_BUTTON);

   public JoystickButton buttonY = new JoystickButton(controller, RobotMap.BUTTON_Y);
   public JoystickButton buttonA = new JoystickButton(controller, RobotMap.BUTTON_A);

  TankDrive drive = new TankDrive();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    climberFront = new Prototype_CAN(RobotMap.CAN_TEST_CLIMBER_FRONT, RobotMap.FRONT_CLIMB_SPEED);
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
    System.out.println("Axis COUNT: " + controller.getAxisCount());
    for (int i=0; i<controller.getAxisCount();i++) {
      System.out.println("Axis Type: " + controller.getAxisType(i));
    }
  }

  /**
   * This function is called periodically during operator control. (approx 20ms)
   */
  @Override
  public void teleopPeriodic() {
    
    double leftSpeed = controller.getRawAxis(RobotMap.LEFT_STICK_Y);
    double rightSpeed = controller.getRawAxis(RobotMap.RIGHT_STICK_Y);

    boolean leftClimberSpeed = buttonY.get();
    boolean rightClimberSpeed = buttonA.get();
    double stickTriggerLeft = controller.getRawAxis(RobotMap.LEFT_TRIGGER);
    double stickTriggerRight = controller.getRawAxis(RobotMap.RIGHT_TRIGGER);
    boolean stickThumbLeft = bumperL.get();
    boolean stickThumbRight = bumperR.get();

    drive.tankDrive(leftSpeed, rightSpeed);

    // The 0.25 and -0.25 are so that the joystick doesn't have to be perfectly
    // centered to stop
    if (leftClimberSpeed ) {
      climberWheelLeft.forward();
      climberWheelRight.forward();
    } else if (rightClimberSpeed) {
      climberWheelLeft.reverse();
      climberWheelRight.reverse();
    } else {
      climberWheelLeft.stop();
      climberWheelRight.stop();
    }

    if (stickTriggerLeft > 0) {
      climberFront.reverse();
    } else if (stickThumbLeft) {
      climberFront.forward();
    } else {
      climberFront.stop();
    }

    if (stickTriggerRight > 0) {
      climberBack.reverse();
    } else if (stickThumbRight) {
      climberBack.forward();
    } else {
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
