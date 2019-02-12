/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class AutoClimb {
    AnalogInput ultrasonic = new AnalogInput(0);


    private double getDistanceInInches() {
        double voltageUltrasonic = ultrasonic.getVoltage();
        double ultrasonicInches = (voltageUltrasonic / RobotMap.ULTRASONIC_VOLTAGE_TO_INCHES);

        SmartDashboard.putNumber("Ultrasonic Voltage: ", voltageUltrasonic);

        return ultrasonicInches;
    }

//ignore all other inputs? override button

}
