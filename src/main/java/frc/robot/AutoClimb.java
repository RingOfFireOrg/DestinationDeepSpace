package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.SwerveDrive;
import static frc.robot.Climber.Location.FRONT;
import static frc.robot.Climber.Location.BACK;

public class AutoClimb {
    private int step = 0;
    private Climber climber;
    private SwerveDrive swerveDrive;

    private DigitalInput frontLeftWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_FRONT_LEFT_WHEEL);
    private DigitalInput backLeftWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_BACK_LEFT_WHEEL);
    private DigitalInput frontRightWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_FRONT_RIGHT_WHEEL);
    private DigitalInput backRightWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_BACK_RIGHT_WHEEL);

    private Timer timer = new Timer();

    private boolean autoClimbFinish = false; //false means not done

    public AutoClimb(Climber climber, SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        timer.reset();
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

    public boolean autoClimbEnabled() {
        if (autoClimbFinish || step == 0) {
            return false;
        } else {
            return true;
        }
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
            climber.extend();

            if(climber.isFullyExtended()) {
                step++;
            }
            //Does the climber ever stop going up???
            break;

        // drive forward until front of robot is on platform and front leg hits platform
        case 3:
            if (frontLeftWheelLimitSwitch.get() || frontRightWheelLimitSwitch.get()) { 
                climber.stopDriving();
                timer.reset();
                timer.start();
            } else {
                climber.driveForward();
                step++;
                //shouldn't this be in the if statement???
            }
            break;    

        //back up for 1/2 a second
        case 4:
            if (timer.get() < 0.5) {
                climber.driveReverse();
            } else {
                climber.stopDriving();
                step++;
            }
            break;

        // lift front leg up all the way
        case 5:    
            if(climber.isFullyRetracted(FRONT)) {
                step++;
            } else {
                climber.retract(FRONT);
            }
            //is the front ever stopped???
           break;

        // drive forward until robot is on platfrom and back leg hits platform AND MOVE CLIMBER BACK UP
        case 6: 
            if (backLeftWheelLimitSwitch.get() || backRightWheelLimitSwitch.get()) { 
                climber.stopDriving(); 
                timer.reset();
                timer.start();
            } else {
                climber.driveForward();
                step++;
                //shouldn't the step be in the if statement???
            }
            break;

        //back up for 1/2 a second
        case 7: 
            if (timer.get() < 0.5) {
                climber.driveReverse();
            } else {
                climber.stopDriving();
                step++;
            }
            break;

        // lift back leg all the way up
        case 8:   
            if(climber.isFullyRetracted(BACK)) {
                timer.reset();
                timer.start();
                step++;
            } else {
                climber.retract(BACK);
                //is the back ever stopped
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