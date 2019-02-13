/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

import frc.robot.SwerveDrive;

/**
 * Add your docs here.
 */
public class AutoClimb {
    private static final int STOP_AT_FRONT_LEG = 12; //find actual distance
    private static final int STOP_AT_BACK_LEG = 12; //find actual distance
    private AnalogInput ultrasonic = new AnalogInput(0);
    private int step = 0;
    private Climber climberFront;
    private Climber climberBack;
    private Climber climberWheelLeft;
    private Climber climberWheelRight;
    private SwerveDrive frontLeft;
    private SwerveDrive frontRight;
    private SwerveDrive backLeft;
    private SwerveDrive backRight;

    public AutoClimb(Climber climberFront, Climber climberBack, Climber climberWheelLeft, Climber climberWheelRight) {
        this.climberFront = climberFront;
        this.climberBack = climberBack;
        this.climberWheelLeft = climberWheelLeft;
        this.climberWheelRight = climberWheelRight;
    }

    private double getDistanceInInches() {
        double voltage = ultrasonic.getVoltage();
        double inInches = (voltage / RobotMap.ULTRASONIC_VOLTAGE_TO_INCHES);

        SmartDashboard.putNumber("Ultrasonic Voltage: ", voltage);

        return inInches;
    }

    private void driveForward() {
        climberWheelLeft.forward();
        climberWheelRight.forward();
    }

    private void stopDriving() {
        climberWheelLeft.stop();
        climberWheelRight.stop();
    }

    public void autoClimb(double inInches) {
        switch(step) {
        case 0:
                //drive back 2 inches
                step++;

            break;
        case 1:
        // if(limitSwitchTop.pressed == false) {
        climberFront.reverse();
        climberBack.reverse();
        // } else {
        //   climberFront.stop();
        //   climberBack.stop();
             step++;
        // }
            break;

        case 2:
            // drive forward x inches(enough where front wheel is on platform but leg is not hitting)
            if (getDistanceInInches() > STOP_AT_FRONT_LEG) { 
                driveForward();
            } else {
                stopDriving();
                step++;
            }

            break;    

        case 3:
            // if(frontLimitSwitchBottom == false)  {
                climberFront.forward();
            // } else {
                climberFront.stop();
                step++;
           // }       
           break;

        case 4: 
        //drive forward x inches (enough where back leg is not hitting platform)
        if (getDistanceInInches() > STOP_AT_BACK_LEG) { 
            driveForward();
        } else {
            stopDriving();
            step++;
        }
        step++;
        break;

        case 5:
             // if(backLimitSwitchBottom == false)  {
             climberBack.forward();
                // } else {
                    climberBack.stop();
                    step++;
                // }       
                break; 
        case 6:
            //drive forward how many ever inches to be fully on platform
        }   

    }

//ignore all other inputs? override button

}