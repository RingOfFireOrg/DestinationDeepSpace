package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CargoManipulator.wheelState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    private double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    private double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    private double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    private double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    private double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    private double tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
    SwerveDrive swerveDrive = new SwerveDrive();

    private boolean cameraFacingBeak;
    private int automationStep = 0;
    Beak beak = new Beak();
    CargoManipulator cargoManipulator = new CargoManipulator();

    private boolean validTarget() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
        thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
        tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);

        if ((tv == 1) && (thor > tvert)) {
            // if there is a target at all and the target's rough bounding box's horizontal side is longer than the vertical side
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
                && cargoManipulator.inShootingPosition()) {
            return true;
        } else {
            return false;
        }
    }

    double steering_adjust = 0.0;

    void aimHatch() {
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        double heading_error = -tx;
        double Kp = -0.1;
        double min_command = 0.1;

        if (tx > .1) {
            // steering_adjust = Kp*heading_error - min_command;
        } else if (tx < .1) {
            // steering_adjust = Kp*heading_error + min_command;
        }
        // left_command += steering_adjust;
        // right_command -= steering_adjust;

         // These numbers must be tuned for your Robot!  Be careful!
         final double STEER_K = 0.03;                    // how hard to turn toward the target
         final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
         final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
         final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
 
         double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
         double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
         double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
         double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
 
         // Start with proportional steering
         double strafeRightLeft = tx * STEER_K;
         swerveDrive.translateAndRotate(0, 0, 0, 0, 0, strafeRightLeft, 0);
         
 
         // try to drive forward until the target area reaches our desired area
         double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
 
         // don't let the robot drive too fast into the goal
         if (drive_cmd > MAX_DRIVE)
         {
           drive_cmd = MAX_DRIVE;
         }
         m_LimelightDriveCommand = drive_cmd;


    }

    void hatchPickup() {
        // automation for getting hatch from feeder station
        /*
         * switch(automationStep){ case 0: if(){ automationStep++; break; } else {
         * break; } case 1: if(){ automationStep++; break; } else { break; } case 2:
         * if(){ automationStep++; break; } else { break; } }
         */

    }

    void hatchScore() {
        // automation for scoring hatch on cargo ship or lower level rocket
    }

    void cargoScore() {
        // automation for scoring hatch on cargo ship or lower level rocket
    }

}
