/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class CargoArmAngler {


//these are in A/D values
//is this here for _______?
    private static final double ARM_TOP = 3351;
    private static final double ARM_BOTTOM = 420;
    private static final double ARM_AT_CARGO_SHIP = 1180;
    private static final double ARM_AT_ROCKET_BOTTOM = 934;

    private AnalogInput cargoEncoder = new AnalogInput(RobotMap.ENCODER_CARGO_LIFT);
    private VictorSP cargoArmMotor = new VictorSP(RobotMap.CARGO_ARM);

    public void printVoltage() {
        SmartDashboard.putNumber("Cargo Encoder Voltage: ", cargoEncoder.getVoltage());
        SmartDashboard.putNumber("Cargo Encoder Value:", cargoEncoder.getVoltage() * 54);
        SmartDashboard.putNumber("Version #", 1);
    }

    public boolean isGoingToTop() {
        if(cargoEncoder.getValue() < ARM_TOP) {
            cargoArmMotor.setSpeed(RobotMap.CARGO_ARM_ROTATION_SPEED);
            return true;
        } else {
            cargoArmMotor.setSpeed(0);
            return false;
        }
    }

    public boolean isAtBottom() {
        if(cargoEncoder.getValue() > ARM_BOTTOM) {
            cargoArmMotor.setSpeed(-RobotMap.CARGO_ARM_ROTATION_SPEED);
            return false;
        } else { 
            cargoArmMotor.setSpeed(0);
            return true;
        }
    }

    public boolean isAtCargoShip() {
        if(cargoEncoder.getValue() > ARM_AT_CARGO_SHIP) {
            cargoArmMotor.setSpeed(-RobotMap.CARGO_ARM_ROTATION_SPEED);
            return false;
        } else if(cargoEncoder.getValue() < ARM_AT_CARGO_SHIP){
            cargoArmMotor.setSpeed(RobotMap.CARGO_ARM_ROTATION_SPEED);
            return false;
        } else {
            cargoArmMotor.setSpeed(0);
            return true;
        }
    }

        public boolean isAtRocketBottom() {
            if(cargoEncoder.getValue() > ARM_AT_ROCKET_BOTTOM) {
                cargoArmMotor.setSpeed(-RobotMap.CARGO_ARM_ROTATION_SPEED);
                return false;
            } else if(cargoEncoder.getValue() < ARM_AT_ROCKET_BOTTOM){
                cargoArmMotor.setSpeed(RobotMap.CARGO_ARM_ROTATION_SPEED);
                return false;
            } else {
                cargoArmMotor.setSpeed(0);
                return true;
            }
            
        
    }
}
