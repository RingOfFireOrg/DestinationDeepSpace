package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CargoManipulator.intakePosition;
import frc.robot.CargoManipulator.wheelState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.kauailabs.navx.frc.AHRS;

public class Vision {
    private double ts;
    private double tv;
    private double tx;
    private double ty;
    private double ta;
    private double thor;
    private double tvert;
    AHRS ahrs;

    private boolean automationRunning = false;

    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to drive sideways to center on the target
    final double DRIVE_K = 0.7; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall

    SwerveDrive swerveDrive = SwerveDrive.getInstance(ahrs);
    Beak beak = Beak.getInstance();
    CargoManipulator cargoManipulator = CargoManipulator.getInstance();
    private boolean cameraFacingBeak;
    private int automationStep = 0;

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
                && cargoManipulator.getPosition() == intakePosition.INTAKE && Math.abs(90 - (ahrs.getCompassHeading() % 90)) < 10) {
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
        double strafeRightLeft = tx * STEER_K * -1;
        double strafeForwardBack = (DESIRED_TARGET_AREA / ta) * DRIVE_K;
        final double MAX_SPEED = 0.4;
        final double MIN_SPEED = 0.1;
        boolean rightLeftAligned = false;
        boolean frontBackAligned = false;
        SmartDashboard.putNumber("translate x", strafeRightLeft);

        if (Math.abs(tx) < 3) { // three is a random placeholder

            // deal with min and max speeds for right left
            if (strafeRightLeft < MIN_SPEED && strafeRightLeft >= 0) {
                strafeRightLeft = MIN_SPEED;
            } else if (strafeRightLeft > -MIN_SPEED) {
                strafeRightLeft = -MIN_SPEED;
            } else if (Math.abs(strafeRightLeft) > MAX_SPEED && strafeRightLeft <= 0) {
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

        if (Math.abs(DESIRED_TARGET_AREA / ta) < 0.9) {
            // this would speed up or slow down and go drive_K for the perfect setup
            // change the division to subtraction
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

        // connor fix this one \/
        swerveDrive.translateAndRotate(0, 0, 0, 0, 0, strafeRightLeft, strafeForwardBack);

        if (frontBackAligned && rightLeftAligned) {
            return true;
        } else {
            return false;
        }

    }

    boolean snapTo90DegreeAngle() { // connor - fix everything in here
        if (ahrs.getCompassHeading() % 90 > 2) {
            if (ahrs.getCompassHeading() < 10 && ahrs.getCompassHeading() > 350) {
                // snap to 0 degrees
                swerveDrive.translateAndRotate(0, 0, 0, 0, 0, 0, 0);
            } else if (ahrs.getCompassHeading() > 80 && ahrs.getCompassHeading() < 100) {
                // snap to 90 degrees
                swerveDrive.translateAndRotate(0, 0, 0, 0, 90, 0, 0);
            } else if (ahrs.getCompassHeading() > 170 && ahrs.getCompassHeading() < 190) {
                // snap to 180 degrees
                swerveDrive.translateAndRotate(0, 0, 0, 0, 180, 0, 0);
            } else if (ahrs.getCompassHeading() > 260 && ahrs.getCompassHeading() < 280) {
                // snap to 270 degrees
                swerveDrive.translateAndRotate(0, 0, 0, 0, 270, 0, 0);
            } else { // if you aren't within the 20 degree window for each 90 degree angle, you are
                     // likely trying to deliver to the rocket OR a lost cause
                automationStep++;
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
        //this will do things in the future
    }

    boolean hatchScore() {
        //this will do things in the future
        return true;
    }

    boolean cargoScoreCargoShip() {
        // automation for getting hatch from feeder station
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
            if (snapTo90DegreeAngle() == false) {
                // nothing happens - keep turning
            } else {
                automationStep++;
            }
            break;
        case 2:// line up with target
            if (alignment() == false) {
                // nothing so running alignment
            } else {
                automationStep++;
            }
            break;
        case 3: // score
            if (!(cargoManipulator.getPosition() == intakePosition.CARGO_SHIP)
                    || cargoManipulator.getAtTargetAngle() == false) {
                cargoManipulator.setToCargoShipPosition();
            } else {
                automationStep++;
            }
            break;
        case 4: // back up slightly
            if (DESIRED_TARGET_AREA / ta > 0.8) {
                //connor fix \/
                swerveDrive.translateAndRotate(0, 0, 0, 0, 0, 0, -2);
            } else {
                automationStep++;
            }
            break;
        case 5:
            break;
        }

        if (automationStep == 5) {
            automationRunning = false;
            automationStep = 0;
            return true;
        } else {
            return false;
        }
    }

    boolean cargoScoreRocket() {
        // automation for getting hatch from feeder station
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        automationRunning = true;

        switch (automationStep) {
        case 0: // set up step -> eventually this will set the pipeline
            swerveDrive.setRobotFrontToHatch();
            automationStep++;
            break;
        case 1: // angle correction
            if (snapTo90DegreeAngle() == false) {
                // nothing happens - keep snapping
            } else {
                automationStep++;
            }
            break;
        case 2:// line up with target
            if (alignment() == false) {
                // nothing so running alignment
            } else {
                automationStep++;
            }
            break;
        case 3: // raise cargo
            if (!(cargoManipulator.getPosition() == intakePosition.LOWER_ROCKET)
                    || cargoManipulator.getAtTargetAngle() == false) {
                cargoManipulator.setToLowerRocketPosition();
            } else {
                automationStep++;
            }
            break;
        case 4:
            break;
        case 5: // back up slightly
            if (DESIRED_TARGET_AREA / ta > 0.8) {
                //connor fix \/
                swerveDrive.translateAndRotate(0, 0, 0, 0, 0, 0, -2);
            } else {
                automationStep++;
            }
            break;
        case 6:
            break;
        }

        if (automationStep == 6) {
            automationRunning = false;
            automationStep = 0;
            return true;
        } else {
            return false;
        }
    }

    boolean isAutomationRunning() {
        return automationRunning;
    }

}
