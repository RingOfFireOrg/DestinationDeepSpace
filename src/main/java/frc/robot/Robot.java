package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private Joystick leftStick;
  private Joystick rightStick;
  private Joystick manipulatorStick;

  private JoystickButton climberPistonButton;
  private JoystickButton buttonLineFollowingEnable;
  private JoystickButton buttonLineFollowingEmergencyStop;

  private Solenoid climbPistonLeft = new Solenoid(RobotMap.CLIMB_PISTON_LEFT_SOLENOID);
  private Solenoid climbPistonRight = new Solenoid(RobotMap.CLIMB_PISTON_RIGHT_SOLENOID);

  private boolean lineFollowing = false;
  private boolean buttonCycle = false;

  private int lineFollowingState = 0;

  private LightSensor leftSensor;
  private LightSensor centerSensor;
  private LightSensor rightSensor;

  TankDrive drive = new TankDrive();

  @Override
  public void robotInit() {

    leftStick = new Joystick(RobotMap.LEFT_DRIVE_JOYSTICK);
    rightStick = new Joystick(RobotMap.RIGHT_DRIVE_JOYSTICK);
    manipulatorStick = new Joystick(RobotMap.MANIPULATOR_DRIVE_JOYSTICK);

    climberPistonButton = new JoystickButton(leftStick, RobotMap.CLIMBER_PISTON_CONTROL);
    buttonLineFollowingEnable = new JoystickButton(rightStick, RobotMap.LINE_FOLLOWING_ENABLE);
    buttonLineFollowingEmergencyStop = new JoystickButton(rightStick, RobotMap.LINE_FOLLOWING_EMERGENCY_STOP);

    leftSensor = new LightSensor(RobotMap.LIGHT_SENSOR1_LIGHT1, RobotMap.LIGHT_SENSOR1_LIGHT2, "left");
    centerSensor = new LightSensor(RobotMap.LIGHT_SENSOR2_LIGHT1, RobotMap.LIGHT_SENSOR2_LIGHT2, "center");
    rightSensor = new LightSensor(RobotMap.LIGHT_SENSOR3_LIGHT1, RobotMap.LIGHT_SENSOR3_LIGHT2, "right");

  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("LeftJoystickY", leftStick.getY());
    SmartDashboard.putNumber("RightJoystickY", rightStick.getY());

    if (buttonLineFollowingEnable.get()) {
      lineFollowing = true;
    } else if (buttonLineFollowingEmergencyStop.get()) {
      lineFollowing = false;
    }
    if (lineFollowing) {
      lineFollowing(rightSensor.get(), centerSensor.get(), leftSensor.get());
    } else {
      drive.tankDrive(-leftStick.getY(), -rightStick.getY());
    }
    SmartDashboard.putBoolean("LineFollowing", lineFollowing);

  }

  public void lineFollowing(boolean right,boolean center, boolean left) {
    if (left == false && center == false && right == true) { lineFollowingState = -2; }
    if (left == false && center == true && right == true) { lineFollowingState = -1; }
    if (left == false && center == true && right == false) { lineFollowingState = 0; }
    if (left == true && center == true && right == false) { lineFollowingState = 1; }
    if (left == true && center == false && right == false) { lineFollowingState = 2; }

    SmartDashboard.putNumber("LFState", lineFollowingState);
    SmartDashboard.putNumber("leftPower", 0.5 - (0.1 * lineFollowingState));
    SmartDashboard.putNumber("leftPower", 0.5 + (0.1 * lineFollowingState));
    drive.tankDrive(0.5 - (0.1 * lineFollowingState), 0.5 + (0.1 * lineFollowingState));
  }

  public void pneumaticsCode() {
    if(buttonCycle && climberPistonButton.get()) {
      //when button pressed, will switch piston state
      if (climbPistonLeft.get() == false) {
        climbPistonLeft.set(true);
        climbPistonRight.set(false);
      } else {
        climbPistonLeft.set(false);
        climbPistonRight.set(true);
      }
      buttonCycle = false;
    } 

    if (climberPistonButton.get() == false) {
      buttonCycle = true;
    }
  }
}
