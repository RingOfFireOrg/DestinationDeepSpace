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

     private boolean foundFirstMagnet = true;

     public void testSensor() {
        if(hallEffectSensor.get()) {
            foundFirstMagnet = false;
            SmartDashboard.putBoolean("Found first magnet? ", foundFirstMagnet);
        } else {
            foundFirstMagnet = true;
            SmartDashboard.putBoolean("Found first magnet? ", foundFirstMagnet);
        }

        if(foundFirstMagnet = true && hallEffectSensor.get()) {
            SmartDashboard.putString("Found second magnet?", "Second magnet found");
        } else {
            SmartDashboard.putString("Found second magnet?", "Second magnet not found");
        }
    }
}

