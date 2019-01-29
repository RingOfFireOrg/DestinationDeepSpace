/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;


/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends TimedRobot {
  Prototype_CAN rPGrabber;
  private Joystick leftStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
  private Joystick rightStick = new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
  private Joystick manipulatorStick = new Joystick(RobotMap.JOYSTICK_MANIPULATOR);
  private JoystickButton hatchGrabberButtonOpening = new JoystickButton(manipulatorStick,
      RobotMap.HATCH_GRABBER_BUTTON_OUT);
  private JoystickButton hatchGrabberButtonClose = new JoystickButton(manipulatorStick, RobotMap.HATCH_GRABBER_BUTTON_IN);

  CAN_TankDrive drive = new CAN_TankDrive();

  Timer hatchGrabberTimer = new Timer();

  Boolean hatchGrabberOpening = false;
  Boolean hatchGrabberClosing = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // crossbow = new Prototype_CAN(RobotMap.CAN_TEST_ATTACHMENT,
    // RobotMap.SPEED_DEFAULT_TEST);
    rPGrabber = new Prototype_CAN(RobotMap.MOTOR_GRABBER, RobotMap.SPEED_DEFAULT_TEST);
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

    drive.tankDrive(leftSpeed, rightSpeed);

    if (hatchGrabberOpening && (hatchGrabberTimer.get() > 1.0)) {
      System.out.println("Opening Stop");
      rPGrabber.stop();
      hatchGrabberOpening = false;
    }

    if (hatchGrabberClosing && (hatchGrabberTimer.get() > 1.0)) {
      System.out.println("Closing Stop");
      rPGrabber.stop();
      hatchGrabberClosing = false;
    }

    if (hatchGrabberButtonOpening.get()) {

      System.out.println("Opening Start");
      hatchGrabberTimer.reset();
      hatchGrabberTimer.start();
      hatchGrabberOpening = true;

      rPGrabber.forward();

    }

    else if (hatchGrabberButtonClose.get()) {
      System.out.println("Closing Start");
      hatchGrabberTimer.reset();
      hatchGrabberTimer.start();
      hatchGrabberClosing = true;

      rPGrabber.reverse();
    }

    // The 0.25 and -0.25 are so that the joystick doesn't have to be perfectly
    // centered to stop
    /*
     * if (yPos > 0.25) { crossbow.forward(); } else if (yPos < -0.25) {
     * crossbow.reverse(); } else { crossbow.stop(); }
     */
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
