/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;


/**
 * Add your docs here.
 */
public class testHE {
     private DigitalInput hallEffectSensor = new DigitalInput(9); 
     private boolean passSensor = true;

     public void testSensor() {
        if(hallEffectSensor.get()) {
            passSensor = false;
            SmartDashboard.putBoolean("Did magnet pass HE? ", passSensor);
        } else {
            passSensor = true;
            SmartDashboard.putBoolean("Did magnet pass HE? ", passSensor);
        }     
    }
}
