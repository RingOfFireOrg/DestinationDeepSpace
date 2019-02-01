/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
    frontTread = new FrontTread(new WPI_TalonSRX(RobotMap.CAN_FRONT_TREAD_A),
        new WPI_TalonSRX(RobotMap.CAN_FRONT_TREAD_B));
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
    double treadSpeed = leftStick.getX();

    drive.tankDrive(leftSpeed, rightSpeed);

    frontTread.set(treadSpeed);
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
