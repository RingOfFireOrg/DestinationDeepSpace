package frc.robot;

import frc.robot.CargoManipulator;
import static frc.robot.Climber.Location.BACK;
import static frc.robot.Climber.Location.FRONT;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;

public class AutoClimb {
    private int step = 0;
    private Climber climber;
    private SwerveDrive swerveDrive;
    private CargoManipulator cargoManipulator;
    private AHRS ahrs;

    private DigitalInput frontLeftWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_FRONT_LEFT_WHEEL);
    private DigitalInput backLeftWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_BACK_LEFT_WHEEL);
    private DigitalInput frontRightWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_FRONT_RIGHT_WHEEL);
    private DigitalInput backRightWheelLimitSwitch = new DigitalInput(RobotMap.INPUT_BACK_RIGHT_WHEEL);

    private Counter counterFL = new Counter(frontLeftWheelLimitSwitch);
    private Counter counterFR = new Counter(frontRightWheelLimitSwitch);
    private Counter counterBL = new Counter(backLeftWheelLimitSwitch);
    private Counter counterBR = new Counter(backRightWheelLimitSwitch);

    private PID robotPitchPID;
    private double pitchOffset;
    private double startingPitch;

    private Timer timer = new Timer();

    private boolean doingAutoClimb = false; //false means not done

    private double climbAngle = -5;

    public AutoClimb(Climber climber, SwerveDrive swerveDrive, AHRS ahrs, CargoManipulator cargoManipulator) {
        this.swerveDrive = swerveDrive;
        this.climber = climber;
        this.ahrs = ahrs;
        this.cargoManipulator = cargoManipulator;
        timer.reset();
        robotPitchPID = new PID(0.02, 0.00005, 0);
        robotPitchPID.setOutputRange(-1, 1);
    }

    public void autoClimbRestart() {
        step = 0;
        timer.reset();
        climber.reset();
        resetLimitSwitches();
        doingAutoClimb = true;
    }

    public boolean autoClimbEnabled() {
        return doingAutoClimb;
    }

    public void stopAutoClimb() {
        doingAutoClimb = false;
    }

    public void autoClimb() {
        if(!doingAutoClimb) {
            return;
        }

        SmartDashboard.putNumber("AutoClimb step: ", step);

        switch(step) {
        
        //set cargo manipulator arm out of way of climber and get ready to climb
        case 0: 
            // cargoManipulator.setToCargoShipPosition();
            // timer.start();
            step++;                
            break;

        // drive back a little 
        case 1:
            cargoManipulator.setToCurrentPosition();
            // if (timer.get() < 0.05) { 
            //     driveSwerve(-0.5);
            // } else {
            //     stopSwerve();
                pitchOffset = ahrs.getPitch();
                SmartDashboard.putNumber("Pitch Offset auto: ", pitchOffset);
                startingPitch = ahrs.getPitch();
                step++;
            //  robotPitchPID.reset();
            // }
            break;

        //makes the front winch taught
        case 2:
            cargoManipulator.setToCurrentPosition();
            if (Math.abs(startingPitch - ahrs.getPitch()) < 3) { //test
                climber.extend(FRONT);
            } else {
                climber.stopClimbing(FRONT);
                startingPitch = ahrs.getPitch();
                 step++;
            }
            break;

        //makes the back winch taught
        case 3:
            cargoManipulator.setToCurrentPosition();
            if (Math.abs(startingPitch - ahrs.getPitch()) < 3) { //test
                climber.extend(BACK);
            } else {
                climber.stopClimbing(BACK);
               // step++;
               step = 12;
            }
            break;

        // extend front and back legs all the way
        case 4:
            cargoManipulator.setToCurrentPosition();
            robotPitchPID.setError((pitchOffset - ahrs.getPitch()) - climbAngle);
            robotPitchPID.update();
            climber.extendLevel(robotPitchPID.getOutput());

            if(climber.isFullyExtendedL3()) {
                resetLimitSwitches();
                step++;
            }
            break;

        // drive forward until front of robot is on platform and front leg hits platform
        case 5:
            cargoManipulator.setToCurrentPosition();
            if (isFrontLegTouching()) { 
                climber.stopDriving();
                step++;
                timer.reset();
                timer.start();
            } else {
                climber.driveForward();
            }
            break;    

        //back up for 1/2 a second
        case 6:
            cargoManipulator.setToCurrentPosition();
            if (timer.get() < 0.05) {
                climber.driveReverse();
            } else {
                climber.stopDriving();
                step++;
            }
            break;

        // retract front leg all the way
        case 7:  
            cargoManipulator.setToCurrentPosition();
            if(climber.isFullyRetracted(FRONT)) {
                resetLimitSwitches();
                step++;
            } else {
                climber.retract(FRONT);
            }
           break;

        // drive forward until robot is on platfrom and back leg hits platform AND MOVE CLIMBER BACK UP
        case 8: 
            if (isBackLegTouching()) { 
                climber.stopDriving(); 
                step++;
                timer.reset();
                timer.start();
            } else {
                climber.driveForward();
            }
            break;

        //back up for 1/2 a second
        case 9: 
            if (timer.get() < 0.05) {
                climber.driveReverse();
            } else {
                climber.stopDriving();
                step++;
            }
            break;

        // retract back leg all the way up
        case 10:   
            if(climber.isFullyRetracted(BACK)) {
                timer.reset();
                timer.start();
                step++;
            } else {
                climber.retract(BACK);
            }
            break;

        // drive forward until all the way on platfrom
        case 11:
            if (timer.get() < 0.5) {
                driveSwerve(0.5);
            } else {
                stopSwerve();
                doingAutoClimb = false;
                step++;
            }
        }   
    }

    private boolean isFrontLegTouching() {
        return counterFL.get() > 0 || counterFR.get() > 0;
    }

    private boolean isBackLegTouching() {
        return counterBL.get() > 0 || counterBR.get() > 0;
    }

    private void resetLimitSwitches() {
        counterFL.reset();
        counterBL.reset();
        counterFR.reset();
        counterBR.reset();
    }
    
    private void driveSwerve(double speed) {
        swerveDrive.syncroDrive(speed, 0, 0, 0);
    }

    private void stopSwerve() {
        swerveDrive.syncroDrive(0, 0, 0, 0);
    }

}