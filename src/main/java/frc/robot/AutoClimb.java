/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.getEncoderValues;

import frc.robot.SwerveDrive;

/**
 * Add your docs here.
 */
public class AutoClimb {
    AnalogInput ultrasonic = new AnalogInput(0);
    int step = 0;
    Climber climberFront;
    Climber climberBack;
    Climber climberWheelLeft;
    Climber climberWheelRight;
    SwerveDrive frontLeft;
    SwerveDrive frontRight;
    SwerveDrive backLeft;
    SwerveDrive backRight;
    

    private double getDistanceInInches() {
        double voltage = ultrasonic.getVoltage();
        double inInches = (voltage / RobotMap.ULTRASONIC_VOLTAGE_TO_INCHES);

        SmartDashboard.putNumber("Ultrasonic Voltage: ", voltage);

        return inInches;
    }

    private double getEncoderValues() {
        double frontLD = frontLeft.getDistance();
        double frontRD = frontRight.getDistance();
        double backLD = backLeft.getDistance();
        double backRD = backRight.getDistance();

        return frontLD;
        return frontRD;
        return backLD;
        return backRD;
    }

    public void autoClimb(double inInches) {
        
        case 0:
            if (inInches <= 6) {
                
                step++;
                break;
            }
		case 1:
			if(step == 1) {
                
            }
			
    }



//ignore all other inputs? override button

}
