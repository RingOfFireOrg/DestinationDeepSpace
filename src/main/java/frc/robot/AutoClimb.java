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
    int step = 0;
    TankDrive drive;
    Prototype_CAN climberFront;
    Prototype_CAN climberBack;
    Prototype_CAN climberWheelLeft;
    Prototype_CAN climberWheelRight;
    

    private double getDistanceInInches() {
        double voltage = ultrasonic.getVoltage();
        double inInches = (voltage / RobotMap.ULTRASONIC_VOLTAGE_TO_INCHES);

        SmartDashboard.putNumber("Ultrasonic Voltage: ", voltage);

        return inInches;
    }

    public void autoClimb(double inInches) {
        /*
        case 0:
            if (inInches <= 6) {
                drive.tankDrive(-0.75, -0.75); //change to drive certain wheel rotation or distance
                step++;
                break;
            }
		case 1:
			
			*/
    }



//ignore all other inputs? override button

}
