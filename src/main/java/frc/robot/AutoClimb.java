/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.SwerveDrive;

/**
 * Add your docs here.
 */
public class AutoClimb {
    private int step = 0;
    private Climber climberFront;
    private Climber climberBack;
    private Climber climberWheelLeft;
    private Climber climberWheelRight;
    private SwerveDrive swerveDrive;

    private DigitalInput frontHallEffectTop = new DigitalInput(RobotMap.INPUT_FRONT_TOP_SW);
    private DigitalInput backHallEffectTop = new DigitalInput(RobotMap.INPUT_BACK_TOP_SW);
    private DigitalInput frontHallEffectBottom = new DigitalInput(RobotMap.INPUT_FRONT_BOTTOM_SW);
    private DigitalInput backHallEffectBottom = new DigitalInput(RobotMap.INPUT_BACK_BOTTOM_SW);

    private DigitalInput frontLeftWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_FRONT_LEFT_WHEEL);
    private DigitalInput backLeftWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_BACK_LEFT_WHEEL);
    private DigitalInput frontRightWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_FRONT_RIGHT_WHEEL);
    private DigitalInput backRightWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_BACK_RIGHT_WHEEL);

    private Timer timer = new Timer();

    private boolean autoClimbFinish = false; //false means not done

    public AutoClimb(Climber climberFront, Climber climberBack, Climber climberWheelLeft, Climber climberWheelRight, SwerveDrive swerveDrive) {
        this.climberFront = climberFront;
        this.climberBack = climberBack;
        this.climberWheelLeft = climberWheelLeft;
        this.climberWheelRight = climberWheelRight;
        this.swerveDrive = swerveDrive;
        timer.reset();
    }

    private void driveClimberWheelsForward() {
        climberWheelLeft.forward();
        climberWheelRight.forward();
    }

    private void driveClimberWheelsReverse() {
        climberWheelLeft.reverse();
        climberWheelRight.reverse();
    }

    private void stopClimberWheelsDriving() {
        climberWheelLeft.stop();
        climberWheelRight.stop();
    }

    private void driveSwerve(double speed) {
        swerveDrive.syncroDrive(speed, 0, 0, 0);
    }

    private void stopSwerve() {
        swerveDrive.syncroDrive(0, 0, 0, 0);
    }

    public void autoClimbInit() {
        step = 0;
        timer.reset();
        autoClimbFinish = false;
    }

    public boolean autoClimbFinished() {

        switch(step) {
        
        case 0: 
            timer.start();
            step++;
            break;

        // drive back a little
        case 1:
                if (timer.get() < 0.5) {
                    driveSwerve(-0.5);
                } else {
                    stopSwerve();
                    step++;
                }
            break;

        // raise front and back legs all the way
        case 2:
            if(frontHallEffectTop.get()) {
                climberFront.stop();
            } else {
                climberFront.forward();
            }

            if(backHallEffectTop.get()) {
                climberBack.stop();
            } else {
                climberBack.forward();
            }

            if(frontHallEffectTop.get() && backHallEffectTop.get()) {
                step++;
            }
            break;

        // drive forward until front of robot is on platform but not hitting front leg
        case 3:
            if (frontLeftWheelLimitSwitch.get() || frontRightWheelLimitSwitch.get()) { 
                stopClimberWheelsDriving();
                timer.reset();
                timer.start();
            } else {
                driveClimberWheelsForward();
                step++;
            }
            break;    

        case 4:
            if (timer.get() < 0.5) {
                driveClimberWheelsReverse();
            } else {
                stopClimberWheelsDriving();
                step++;
            }
            break;

        // lift front leg up all the way
        case 5:    
            if(frontHallEffectBottom.get()) {
                climberFront.stop();
                step++;
            } else {
                climberFront.reverse();
            }
           break;

        // drive forward until robot is on platfrom till right before back leg
        case 6: 
            if (backLeftWheelLimitSwitch.get() || backRightWheelLimitSwitch.get()) { 
                stopClimberWheelsDriving();
                timer.reset();
                timer.start();
            } else {
                driveClimberWheelsForward();
                step++;

            }
            break;

        case 7: 
            if (timer.get() < 0.5) {
                driveClimberWheelsReverse();
            } else {
                stopClimberWheelsDriving();
                step++;
            }
            break;

        // lift back leg all the way up
        case 8:   
            if(backHallEffectBottom.get()) {
                climberBack.stop();
                timer.reset();
                timer.start();
                step++;
            } else {
                climberBack.reverse();
            }
            break;

        // drive forward until all the way on platfrom
        case 9:
            if (timer.get() < 2) {
                driveSwerve(0.5);
            } else {
                stopSwerve();
                autoClimbFinish = true;
                step++;
            }
        }   

        return autoClimbFinish;
    }

}