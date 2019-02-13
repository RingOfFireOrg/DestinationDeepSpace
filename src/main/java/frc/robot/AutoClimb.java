/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.SwerveDrive;

/**
 * Add your docs here.
 */
public class AutoClimb {
    private static final int STOP_AT_FRONT_LEG = 12; //find actual distance
    private static final int STOP_AT_BACK_LEG = 12; //find actual distance
    private static final int AGAINST_PLATFORM = 0; //any distance less than 20 inches shows up as 6?
    private static final int DRIVER_STATION_WALL = 0; //find actual distance

    private AnalogInput ultrasonic = new AnalogInput(0);
    private int step = 0;
    private Climber climberFront;
    private Climber climberBack;
    private Climber climberWheelLeft;
    private Climber climberWheelRight;
    private SwerveDrive swerveDrive;

    private DigitalInput frontLimitSwitchTop = new DigitalInput(RobotMap.INPUT_FRONT_TOP_SW);
    private DigitalInput backLimitSwitchTop = new DigitalInput(RobotMap.INPUT_BACK_TOP_SW);
    private DigitalInput frontLimitSwitchBottom = new DigitalInput(RobotMap.INPUT_BACK_BOTTOM_SW);
    private DigitalInput backLimitSwitchBottom = new DigitalInput(RobotMap.INPUT_BACK_BOTTOM_SW);

    public AutoClimb(Climber climberFront, Climber climberBack, Climber climberWheelLeft, Climber climberWheelRight, SwerveDrive swerveDrive) {
        this.climberFront = climberFront;
        this.climberBack = climberBack;
        this.climberWheelLeft = climberWheelLeft;
        this.climberWheelRight = climberWheelRight;
        this.swerveDrive = swerveDrive;
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

    private void driveSwerve() {
        // swerveDrive.translateAndRotate(driveFieldTranslationX, driveFieldTranslationY, unregulatedTurning, gyroReading, fieldRelativeRobotDirection, driveRobotTranslationX, driveRobotTranslationY);

    }

    private void stopSwerve() {
        // stop driving swerve
    }

    public void autoClimb(double inInches) {
        switch(step) {
        
        // drive back a little
        case 0:
                if (getDistanceInInches() <= AGAINST_PLATFORM) {
                    // swerve drive forward
                } else {
                    //swerve drive stop
                    step++;
                }
            break;

        // raise front and back legs all the way
        case 1:
            if(frontLimitSwitchTop.get() && backLimitSwitchTop.get()) {
                climberFront.stop();
                climberBack.stop();
                step++;
            } else {
                climberFront.forward();
                climberBack.forward();
            }
            break;

        // drive forward until front of robot is on platform but not hitting front leg
        case 2:
            if (getDistanceInInches() > STOP_AT_FRONT_LEG) { 
                driveForward();
            } else {
                stopDriving();
                step++;
            }
            break;    

        // lift front leg up all the way
        case 3:    
            if(frontLimitSwitchBottom.get()) {
                climberFront.stop();
                step++;
            } else {
                climberFront.forward();
            }
           break;

        // drive forward until robot is on platfrom till right before back leg
        case 4: 
            if (getDistanceInInches() > STOP_AT_BACK_LEG) { 
                driveForward();
            } else {
                stopDriving();
                step++;
            }
            break;

        // lift back leg all the way up
        case 5:   
            if(backLimitSwitchBottom.get()) {
                climberBack.stop();
                step++;
            } else {
                climberBack.reverse();
            }
            break;

        // drive forward until all the way on platfrom
        case 6:
            //drive swerve how many ever inches to be fully on platform
            if (getDistanceInInches() <= DRIVER_STATION_WALL) {
                // swerve drive forward
            } else {
                //swerve drive stop
                step++;
            }
        }   

    }

//ignore all other inputs? override button

}