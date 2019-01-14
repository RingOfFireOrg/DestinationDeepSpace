/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

Pneumatics piston = new pneumatics(0, 1);

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new VictorSP(2), new VictorSP(1));
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    m_manipulatorStick = new Joystick(2);
    buttonA = new JoystickButton(m_manipulatorStick, 2);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    if(buttonA.getButton()) {
      //when button pressed, will switch piston state
      if (Left1.get() == true) {
        Left1.set(true);
        Right1.set(false);
      } else {
        Left1.set(false);
        Right1.set(true);
      }
    } 

  }
}
