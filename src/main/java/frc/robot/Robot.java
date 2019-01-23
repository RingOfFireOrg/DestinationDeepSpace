/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
 // private DifferentialDrive m_myRobot;
  private Joystick leftStick;
  private Joystick rightStick;
  private Joystick manipulatorStick;
  private JoystickButton buttonA;
  private JoystickButton buttonLineEn;
  private JoystickButton buttonLineES;
  private Solenoid left1 = new Solenoid(0);
  private Solenoid right1 = new Solenoid(1);
  private DigitalInput lightS1L1;
  private DigitalInput lightS1L2;
  private DigitalInput lightS2L1;
  private DigitalInput lightS2L2;
  private DigitalInput lightS3L1;
  private DigitalInput lightS3L2;
  private boolean lineFollowing = false;
  private int lineFollowingState = 0;
  private double lfPowerDifferential = 0;
  Pneumatics piston = new Pneumatics(left1, right1);
  boolean buttonCycle = false;
  //PIDController lineFollowingController = new PIDController(0, 0, 0, lineFollowingState, lfPowerDifferential);

  TankDrive drive = new TankDrive();

  @Override
  public void robotInit() {
    //m_myRobot = new DifferentialDrive(new VictorSP(2), new VictorSP(1));
    leftStick = new Joystick(RobotMap.LEFT_DRIVE_JOYSTICK);
    rightStick = new Joystick(RobotMap.RIGHT_DRIVE_JOYSTICK);
    manipulatorStick = new Joystick(0);
    buttonA = new JoystickButton(leftStick, 1);
    buttonLineEn = new JoystickButton(rightStick, 1);
    buttonLineES = new JoystickButton(rightStick, 2);
    lightS1L1 = new DigitalInput(10);
    lightS1L2 = new DigitalInput(11);
    lightS2L1 = new DigitalInput(12);
    lightS2L2 = new DigitalInput(13);
    lightS3L1 = new DigitalInput(18);
    lightS3L2 = new DigitalInput(19);

  }

  @Override
  public void teleopPeriodic() {
    

    //SmartDashboard.putNumber("Joystick X value", leftStick.getX());
    SmartDashboard.putBoolean("LightS1L1", lightS1L1.get());
    SmartDashboard.putBoolean("LightS1L2", lightS1L2.get());
    SmartDashboard.putBoolean("LightS2L1", lightS2L1.get());
    SmartDashboard.putBoolean("LightS2L2", lightS2L2.get());
    SmartDashboard.putBoolean("LightS3L1", lightS3L1.get());
    SmartDashboard.putBoolean("LightS3L2", lightS3L2.get());

    SmartDashboard.putNumber("LeftJoystickY", leftStick.getY());
    SmartDashboard.putNumber("RightJoystickY", rightStick.getY());

    if (buttonLineEn.get()) {
      lineFollowing = true;
    } else if (buttonLineES.get()) {
      lineFollowing = false;
    }
    if (lineFollowing) {
      lineFollowing(lightS1L1.get(), lightS2L1.get(), lightS3L1.get());
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
    if(buttonCycle && buttonA.get()) {
      //when button pressed, will switch piston state
      if (left1.get() == false) {
        left1.set(true);
        right1.set(false);
      } else {
        left1.set(false);
        right1.set(true);
      }
      buttonCycle = false;
    } 

    if (buttonA.get() == false) {
      buttonCycle = true;
    }
  }
}
