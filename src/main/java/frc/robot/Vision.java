package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CargoManipulator.intakePosition;
import frc.robot.CargoManipulator.wheelState;
import frc.robot.SwerveDrive.selectiveSwerveDriveModes;

public class Vision {
    private double ts;
    private double tv;
    private double tx;
    private double ty;
    private double ta;
    private double thor;
    private double tvert;
    AHRS ahrs = Robot.getGyroInstance();

    private boolean automationRunning = false;
    final int ALLOWED_OFFSET = 20;

    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to drive sideways to center on the target
    final double DRIVE_K = 0.7; // how hard to drive fwd toward the target
    //does this come back in % or decimal form
    final double DESIRED_TARGET_AREA = 13.0;
    final double MAX_SPEED = 0.4;
    final double MIN_SPEED = 0.1; // stall speed?? // Area of the target when the robot reaches the wall 
    

    private PID strafePID;
    private PID drivePID;

    SwerveDrive swerveDrive = SwerveDrive.getInstance(ahrs);
    Beak beak = Beak.getInstance();
    CargoManipulator cargoManipulator = CargoManipulator.getInstance();
    private boolean cameraFacingBeak;
    private int automationStep = 0;

    public double alignmentPositionAngle = 0;

    Vision() {
        strafePID = new PID(STEER_K, 0, 0);
        strafePID.setOutputRange(-MAX_SPEED, MAX_SPEED);
        strafePID.setInternalRestiction(MIN_SPEED);
        drivePID = new PID(DRIVE_K, 0, 0);
        drivePID.setOutputRange(-MAX_SPEED, MAX_SPEED);
        drivePID.setInternalRestiction(MIN_SPEED);

    }
    private boolean validTarget() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
        thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
        tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);

        if ((tv == 1) && (thor > tvert)) {
            // if there is a target at all and the target (two targets) is wider than it is
            // tall
            return true;
        } else {
            return false;
        }
    }

    void swtichCameraToBeakSide() {
        // TODO Physically switch camera
        cameraFacingBeak = true;
    }

    void switchCameraToCargoManipulator() {
        // TODO Physically switch camera
        cameraFacingBeak = false;
    }

    boolean hatchPickupReady() {
        if (validTarget() && !beak.isOpen() && cameraFacingBeak) {
            return true;
        } else {
            return false;
        }
    }

    boolean hatchScoreReady() {
        if (validTarget() && beak.isOpen() && cameraFacingBeak) {
            return true;
        } else {
            return false;
        }
    }

    boolean cargoScoreReady() {
        if (validTarget() && cargoManipulator.getWheelState() == wheelState.OFF && !cameraFacingBeak
                && cargoManipulator.getPosition() == intakePosition.INTAKE
                && Math.abs(90 - (ahrs.getCompassHeading() % 90)) < 10) {
            return true;
        } else {
            return false;
        }
    }

    void leftRightAlignmentTest() {
        String leftRightStop;
        if (Math.abs(tx) < 3) { // three is a random placeholder
            double strafeRightLeft = tx * STEER_K * -1;
            SmartDashboard.putNumber("translate x", strafeRightLeft);

            if (strafeRightLeft >= 0) {
                leftRightStop = "left";
            } else { // if(strafeRightLeft < 0)
                leftRightStop = "right";
            }

        } else {
            leftRightStop = "stop";
        }

        SmartDashboard.putString("Strafe direction left/right", leftRightStop);
    }

    boolean alignment() {
        //Alternative PID framework (written on 4/1/19, might not have been updated) ------>
        // strafePID.setError(-tx);
        // drivePID.setError(1 - (DESIRED_TARGET_AREA / ta));
        // strafePID.update();
        // drivePID.update();
        // boolean xAligned = false, yAligned = false;
        // SmartDashboard.putNumber("XTranslation", strafePID.getOutput());
        // double translationX = strafePID.getOutput();
        // double translationY = drivePID.getOutput();
        // if (Math.abs(tx) < 3) {
        //     translationX = 0;
        //     xAligned = true;
        // }
        // if (Math.abs(DESIRED_TARGET_AREA / ta) > 0.9) {
        //     translationY = 0;
        //     yAligned = true;
        // }
        // swerveDrive.selectiveTranslateAndRotate(selectiveSwerveDriveModes.ROBOT_ABSOLUTE, alignmentPositionAngle, translationX, translationY);
        // if(xAligned && yAligned) {
        //     return true;
        // } else {
        //     return false;
        // }

        //if using the PID framework, delete code below: ----->

        double strafeRightLeft = tx * STEER_K * -1;
        double strafeForwardBack = (1 - (DESIRED_TARGET_AREA / ta)) * DRIVE_K;
        boolean rightLeftAligned = false;
        boolean frontBackAligned = false;
        SmartDashboard.putNumber("translate x", strafeRightLeft);

        if (Math.abs(tx) > 3) { // three is a random placeholder
            // RJC is it imprtant that the 3 placeholder here and the 3 placeholder above
            // have the same value? If so maybe move to a constant and use that? if not this is fine.

            // deal with min and max speeds for right left
            if (Math.abs(strafeRightLeft) < MIN_SPEED) {
                if (strafeRightLeft >= 0) {
                    strafeRightLeft = MIN_SPEED;
                } else { // if (strafeRightLeft <= 0
                    strafeRightLeft = -MIN_SPEED;
                }
            } else if (Math.abs(strafeRightLeft) > MAX_SPEED) {
                if (strafeRightLeft > 0) {
                    strafeRightLeft = MAX_SPEED;
                } else {
                    strafeRightLeft = -MAX_SPEED;
                }
            }
        } else {
            strafeRightLeft = 0;
            rightLeftAligned = true;
        }

        //what if you are closer than desired when starting?
        if (Math.abs(DESIRED_TARGET_AREA / ta) < 0.9) { // is 0.9 close enough?? //wouldn't the desired target area be where we stop, so shouldn't this be 1
            if (strafeForwardBack < MIN_SPEED) {
                // won't ever go backward
                strafeForwardBack = MIN_SPEED;
            } else if (strafeForwardBack > MAX_SPEED) {
                strafeForwardBack = MAX_SPEED;
            }
        } else {
            strafeForwardBack = 0;
            frontBackAligned = true;
        }

        swerveDrive.selectiveTranslateAndRotate(selectiveSwerveDriveModes.ROBOT_ABSOLUTE, alignmentPositionAngle, strafeRightLeft, strafeForwardBack);

        if (frontBackAligned && rightLeftAligned) {
            return true;
        } else {
            return false;
        }
        //^
        //|
        //deleted section if using PID ends here _-_-_-

    }

    boolean snapTo90DegreeAngle() {
        if (ahrs.getCompassHeading() % 90 > 2 && ahrs.getCompassHeading() % 90 < 88) {
            if (ahrs.getCompassHeading() < ALLOWED_OFFSET && ahrs.getCompassHeading() > (360 - ALLOWED_OFFSET)) {
                // snap to 0 degrees
                swerveDrive.selectiveTranslateAndRotate(selectiveSwerveDriveModes.ROBOT_ABSOLUTE, 0, 0, 0);
                alignmentPositionAngle = 0;
            } else if (ahrs.getCompassHeading() > (90 - ALLOWED_OFFSET)
                    && ahrs.getCompassHeading() < (90 + ALLOWED_OFFSET)) {
                // snap to 90 degree-s
                swerveDrive.selectiveTranslateAndRotate(selectiveSwerveDriveModes.ROBOT_ABSOLUTE, 90, 0, 0);
                alignmentPositionAngle = 90;
            } else if (ahrs.getCompassHeading() > (180 - ALLOWED_OFFSET)
                    && ahrs.getCompassHeading() < (180 + ALLOWED_OFFSET)) {
                // snap to 180 degrees
                swerveDrive.selectiveTranslateAndRotate(selectiveSwerveDriveModes.ROBOT_ABSOLUTE, 180, 0, 0);
                alignmentPositionAngle = 180;
            } else if (ahrs.getCompassHeading() > (270 - ALLOWED_OFFSET)
                    && ahrs.getCompassHeading() < (270 + ALLOWED_OFFSET)) {
                // snap to 270 degrees
                swerveDrive.selectiveTranslateAndRotate(selectiveSwerveDriveModes.ROBOT_ABSOLUTE, 270, 0, 0);
                alignmentPositionAngle = 270;
            } else { // if you aren't within the 20 degree window for each 90 degree angle, you are
                     // likely trying to deliver to the rocket OR a lost cause
                // question: should we be returning true??? or something else? maybe it should
                // break the automation?
            }
            return false;
        } else {
            return true;
        }
    }

    boolean hatchPickup() {
        return true;
        // this will do things in the future
    }

    boolean hatchScore() {
        // this will do things in the future
        return true;
    }

    boolean cargoScore() {
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        automationRunning = true;

        switch (automationStep) {
        case 0: // set up step -> eventually this will set the pipeline
            swerveDrive.setRobotFrontToCargo();
            automationStep++;
            break;

        case 1: // angle correction
            if (snapTo90DegreeAngle()) {
                automationStep++;
            }
            break;
        case 2:// line up with target
            if (alignment()) {
                automationStep++;
            }
            break;
        case 3: // score
            // if (cargoManipulator.getPosition() != position || !cargoManipulator.getAtTargetAngle()) {
            //     if (position == intakePosition.CARGO_SHIP) {
            //         cargoManipulator.setToCargoShipPosition();
            //     } else if (position == intakePosition.LOWER_ROCKET) {
            //         cargoManipulator.setToLowerRocketPosition();
            //     }
            // } else {
            //     automationStep++;
            // }
            automationStep++;
            break;
        case 4: // back up slightly
            if (DESIRED_TARGET_AREA / ta > 0.8) { // NEED TEST!!
                swerveDrive.selectiveTranslateAndRotate(selectiveSwerveDriveModes.ROBOT_UNREGULATED, 0, 0, -0.2);
            } else {
                automationStep++;
            }
            break;
        case 5:
            automationRunning = false;
            automationStep = 0;
            return true;
            // break;
        }
        return false;
    }

    boolean isAutomationRunning() {
        return automationRunning;
    }

    void logValues(){
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        SmartDashboard.putNumber("tv", tv);
        SmartDashboard.putNumber("ta", ta);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("tx", tx);

    }

}
