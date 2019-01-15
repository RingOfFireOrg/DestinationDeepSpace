/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private Joystick m_manipulatorStick;
  private JoystickButton buttonA;
  private Solenoid left1 = new Solenoid(0);
  private Solenoid right1 = new Solenoid(1);
  Pneumatics piston = new Pneumatics(left1, right1);
  boolean buttonCycle = false;

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new VictorSP(2), new VictorSP(1));
    m_leftStick = new Joystick(1);
    m_rightStick = new Joystick(2);
    m_manipulatorStick = new Joystick(0);
    buttonA = new JoystickButton(m_rightStick, 1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

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
