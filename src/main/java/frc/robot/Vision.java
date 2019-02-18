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
        if (validTarget() && !beak.isOpen() && cameraFacingBeak && beak.isOut()) {
            return true;
        } else {
            return false;
        }
    }

    boolean hatchScoreReady() {
        if (validTarget() && beak.isOpen() && cameraFacingBeak && beak.isOut()) {
            return true;
        } else {
            return false;
        }
    }

    boolean cargoScoreReady() {
        if (validTarget() && cargoManipulator.getWheelState() == wheelState.OFF && !cameraFacingBeak
                && cargoManipulator.getPosition() == intakePosition.INTAKE) {
            return true;
        } else {
            return false;
        }
    }

    boolean hatchPickup() {
        // automation for getting hatch from feeder station
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        automationRunning = true;

        switch (automationStep) {
        case 0:
            swerveDrive.setRobotFrontToHatch();
            automationStep++;
            break;
        case 1: // angle correction
            if (!(ahrs.getCompassHeading() % 90 < 2)) {
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
                } else {
                    automationStep++;
                }
            } else {
                automationStep++;
            }
            break;
        case 2:// strafe right and left to line up with target
            if (Math.abs(tx) < 3) { // three is a random placeholder
                double strafeRightLeft = tx * STEER_K * -1;
                if (strafeRightLeft < .2) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, 0, .2, 0);
                } else if (strafeRightLeft > .7) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, 0, .6, 0);
                } else {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, 0, strafeRightLeft, 0);
                }
            } else {
                automationStep++;
            }
            break;
        case 3: // drive forward to get into intake position
            if (Math.abs(DESIRED_TARGET_AREA / ta) < 0.9) {
                double driveForward = (DESIRED_TARGET_AREA / ta) * DRIVE_K;
                if (driveForward < .2) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, .2, 0, 0);
                } else if (driveForward > .7) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, .6, 0, 0);
                } else {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, driveForward, 0, 0);
                }
            } else {
                automationStep++;
            }
            break;
        case 4: // score
            if (!beak.isOpen()) {
                beak.open();
            } else {
                automationStep++;
            }
            break;
        case 5: // back up slightly
            if (DESIRED_TARGET_AREA / ta > 0.8) {
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
            return true;
        } else {
            return false;
        }
    }

    boolean hatchScore() {
        // automation for getting hatch from feeder station
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        automationRunning = true;

        switch (automationStep) {
        case 0:
            swerveDrive.setRobotFrontToHatch();
            automationStep++;
            break;
        case 1: // angle correction
            if (!(ahrs.getCompassHeading() % 90 < 2)) {
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
                } else {
                    automationStep++;
                }
            } else {
                automationStep++;
            }
            break;
        case 2:// strafe right and left to line up with target
            if (Math.abs(tx) < 3) { // three is a random placeholder
                double strafeRightLeft = tx * STEER_K * -1;
                if (strafeRightLeft < .2) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, 0, .2, 0);
                } else if (strafeRightLeft > .7) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, 0, .6, 0);
                } else {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, 0, strafeRightLeft, 0);
                }
            } else {
                automationStep++;
            }
            break;
        case 3: // drive forward to get into intake position
            if (Math.abs(DESIRED_TARGET_AREA / ta) < 0.9) {
                double driveForward = (DESIRED_TARGET_AREA / ta) * DRIVE_K;
                if (driveForward < .2) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, .2, 0, 0);
                } else if (driveForward > .7) {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, .6, 0, 0);
                } else {
                    swerveDrive.translateAndRotate(0, 0, 0, 0, driveForward, 0, 0);
                }
            } else {
                automationStep++;
            }
            break;
        case 4: // score
            if (beak.isOpen()) {
                beak.close();
            } else {
                automationStep++;
            }
            break;
        case 5: // back up slightly
            if (DESIRED_TARGET_AREA / ta > 0.8) {
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
            return true;
        } else {
            return false;
        }
    }

    boolean cargoScoreCargoShip() {
        // automation for scoring cargo in a cargoShip
        if (automationStep == 6) {
            return true;
        } else {
            return false;
        }
    }

    boolean cargoScoreRocket() {
        if (automationStep == 6) {
            return true;
        } else {
            return false;
        }
    }

    boolean isAutomationRunning(){
        return automationRunning;
    }

}
