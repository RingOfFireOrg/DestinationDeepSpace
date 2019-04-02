/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
  private Joystick leftStick = new Joystick(RobotMap.JOYSTICK_DRIVE_LEFT);
  private Joystick rightStick = new Joystick(RobotMap.JOYSTICK_DRIVE_RIGHT);
  private Joystick manipulatorStick = new Joystick(RobotMap.JOYSTICK_MANIPULATOR);

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

  }

  @Override
  public void teleopPeriodic() {
    double leftSpeed = -leftStick.getY();
    double rightSpeed = -rightStick.getY();
    double yPos = manipulatorStick.getY();

    drive.tankDrive(leftSpeed, rightSpeed);

    if (yPos > 0.25) {
      hatchPrototype.forward();
    } else if(yPos < 0.25) {
      hatchPrototype.reverse();
    } else {
      hatchPrototype.stop();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
