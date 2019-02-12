package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision{
   static double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
   static double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

   static boolean validTarget(){
       tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
       ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
       if((tv == 1) && ((ts >= 13 && ts <=16) || (ts <= -13 && ts >= -16))){
           // if there is a target at all and the target is at an angle the vision targets are at
           return true;
       } else {
           return false;
       }
   }

   static void logAngle(){
       tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
       ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
       if(tv == 1){
           SmartDashboard.putNumber("skew of target", ts);
       }
   }


}
