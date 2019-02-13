package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CargoManipulator.wheelState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision{
   private static double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
   private static double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
   private static boolean cameraFacingBeak;
   private static int automationStep = 0;

   private static boolean validTarget(){
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
   static void logAngle(){
       tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
       ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
       if(tv == 1){
           SmartDashboard.putNumber("skew of target", ts);
       }
   }

   static void swtichCameraToBeakSide(){
        //TODO Physically switch camera
        cameraFacingBeak = true;
   }

   static void switchCameraToCargoManipulator(){
       //TODO Physically switch camera
       cameraFacingBeak = false;
   }

   static boolean hatchPickupReady(){
        if(validTarget() && !Beak.isOpen() && cameraFacingBeak){
            return true;
        } else {
            return false;
        }
   }

   static boolean hatchScoreReady(){
        if(validTarget() && Beak.isOpen()&& cameraFacingBeak){
            return true;
        } else {
            return false;
        }
   }

   static boolean cargoScoreReady(){
        if(validTarget() && CargoManipulator.getWheelState() == wheelState.OFF && !cameraFacingBeak && CargoManipulator.inShootingPosition()){
            return true;
        } else {
            return false;
        }
   }

   static void hatchPickup(){
       // automation for getting hatch from feeder station
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

   }

   static void hatchScore(){
       //automation for scoring hatch on cargo ship or lower level rocket
   }

   static void cargoScore(){
       //automation for scoring hatch on cargo ship or lower level rocket
   }


}
