package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CargoManipulator.wheelState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision{
   private double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
   private double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
   private double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
   private boolean cameraFacingBeak;
   private int automationStep = 0;
   Beak beak = new Beak();
   CargoManipulator cargoManipulator = new CargoManipulator();


   private boolean validTarget(){
       tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
       ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
       if((tv == 1) && ((ts >= 13 && ts <=16) || (ts <= -13 && ts >= -16))){
           // if there is a target at all and the target is at an angle the vision targets are at
           return true;
       } else {
           return false;
       }
   }

   //this method will eventually dissapear. currently using for understanding how angle works
   void logAngle(){
       tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
       ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
       if(tv == 1){
           SmartDashboard.putNumber("skew of target", ts);
       }
   }

   void swtichCameraToBeakSide(){
        //TODO Physically switch camera
        cameraFacingBeak = true;
   }

   void switchCameraToCargoManipulator(){
       //TODO Physically switch camera
       cameraFacingBeak = false;
   }

   boolean hatchPickupReady(){
        if(validTarget() && !beak.isOpen() && cameraFacingBeak){
            return true;
        } else {
            return false;
        }
   }

   boolean hatchScoreReady(){
        if(validTarget() && beak.isOpen() && cameraFacingBeak){
            return true;
        } else {
            return false;
        }
   }

   boolean cargoScoreReady(){
        if(validTarget() && cargoManipulator.getWheelState() == wheelState.OFF && !cameraFacingBeak && cargoManipulator.inShootingPosition()){
            return true;
        } else {
            return false;
        }
   }


   
  
   double steering_adjust = 0.0;

   void aimHatch(){
        
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double heading_error = -tx;
    double Kp = -0.1;
    double min_command = 0.1;

        if (tx > .1){
            //steering_adjust = Kp*heading_error - min_command;
        } else if (tx < .1) {
            //steering_adjust = Kp*heading_error + min_command;
        }
        //left_command += steering_adjust;
        //right_command -= steering_adjust;
   }




   void hatchPickup(){
       // automation for getting hatch from feeder station
       /*
       switch(automationStep){   
        case 0:
            if(){
                automationStep++;
                break;
            } else {
                break;
            }
           case 1:
            if(){
                automationStep++;
                break;
            } else {
                break;
            }
           case 2:
            if(){
                automationStep++;
                break;
            } else {
                break;
            }
       }
       */

   }

   void hatchScore(){
       //automation for scoring hatch on cargo ship or lower level rocket
   }

   void cargoScore(){
       //automation for scoring hatch on cargo ship or lower level rocket
   }


}
