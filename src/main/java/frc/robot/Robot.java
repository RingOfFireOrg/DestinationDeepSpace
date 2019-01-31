/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;

/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends TimedRobot {
  private Joystick leftStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
  private Joystick rightStick = new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
  private Joystick manipulatorStick = new Joystick(RobotMap.JOYSTICK_MANIPULATOR);
  //private Lifter lifter;
  private FrontTread frontTread;

  private Victor leftVictor = new Victor(RobotMap.MOTOR_LEFT);
  private Victor rightVictor = new Victor(RobotMap.MOTOR_RIGHT);
  TankDrive drive = new TankDrive(leftVictor, rightVictor);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    //crossbow = new Prototype_CAN(RobotMap.CAN_TEST_ATTACHMENT, RobotMap.SPEED_DEFAULT_TEST);
    //lifter = new Lifter();
    frontTread = new FrontTread(new TalonSRX(7), new TalonSRX(10));
    leftVictor.setInverted(false);
    rightVictor.setInverted(false);
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
    double yPos = manipulatorStick.getY();

    drive.tankDrive(leftSpeed, rightSpeed);

    boolean upPressed = manipulatorStick.getRawButton(RobotMap.LIFT_UP_BUTTON);
    boolean downPressed = manipulatorStick.getRawButton(RobotMap.LIFT_DOWN_BUTTON);

    frontTread.driveForward(leftStick.getX());
    // if (upPressed) {
    //   lifter.up();
    // } else if (downPressed) {
    //   lifter.down();
    // } else {
    //   lifter.stop();
    // }

    // The 0.25 and -0.25 are so that the joystick doesn't have to be perfectly
    // centered to stop
    // if (yPos > 0.25) {
    //   crossbow.forward(0.95);
    // } else if (yPos < -0.25) {
    //   crossbow.reverse(0.35);
    // } else {
    //   crossbow.stop();
    // }

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
