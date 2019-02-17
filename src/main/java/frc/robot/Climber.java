/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Climber.Location.FRONT;
import static frc.robot.Climber.Location.BACK;


/**
 * Stuff for climber
 */
public class Climber {
/**
     * The default speed to make controlling easier
     */

    private double defaultDriveSpeed;
    private double defaultClimbSpeed;
    private boolean foundFrontTop = false;
    private boolean foundBackTop = false;
    private boolean foundFrontBottom = false;
    private boolean foundBackBottom = false;

    private TalonSRX climberFront = new TalonSRX(RobotMap.CAN_CLIMBER_FRONT);
    private TalonSRX climberBack = new TalonSRX(RobotMap.CAN_CLIMBER_BACK);
    private VictorSP climberWheels = new VictorSP(RobotMap.CAN_CLIMBER_WHEELS);

    private DigitalInput frontHallEffectTop = new DigitalInput(RobotMap.INPUT_FRONT_TOP_SW);
    private DigitalInput backHallEffectTop = new DigitalInput(RobotMap.INPUT_BACK_TOP_SW);
    private DigitalInput frontHallEffectBottom = new DigitalInput(RobotMap.INPUT_FRONT_BOTTOM_SW);
    private DigitalInput backHallEffectBottom = new DigitalInput(RobotMap.INPUT_BACK_BOTTOM_SW);

    /**
     * The name of the object (for use in debug)
     */
    private String name;

    /**
     * The constructor for our class
     * 
     * @param canPort      - which port on the roboRio it is connectedTo
     * @param defaultSpeed - what speed should be used for forward and reverse
     */
    Climber(double defaultDriveSpeed, double defaultClimbSpeed) {
        this.defaultDriveSpeed = defaultDriveSpeed;
        this.defaultClimbSpeed = defaultClimbSpeed;
    }


    public void up() {
        up(FRONT);
        up(BACK);
    }

    public void up(Location loc) {
        if (loc == FRONT && !foundFrontTop) {
            climberFront.set(ControlMode.PercentOutput, -defaultClimbSpeed);
            foundFrontBottom = false;
        } else if(loc == BACK && !foundBackTop) {
            climberBack.set(ControlMode.PercentOutput, -defaultClimbSpeed);
            foundBackBottom = false;
        }
    }

    public void down() {
        down(FRONT);
        down(BACK);
    }

    public void down(Location loc) {
        if (loc == BACK && !foundBackBottom) {
            climberBack.set(ControlMode.PercentOutput, defaultClimbSpeed);
            foundBackTop = false;
        } else if(loc == FRONT && !foundFrontBottom) {
            climberFront.set(ControlMode.PercentOutput, defaultClimbSpeed);
            foundFrontTop = false;
        }
    }

    public void stopClimbing() {
        stopClimbing(FRONT);
        stopClimbing(BACK);
    }

    public void stopClimbing(Location loc) {
        if (loc == FRONT) {
            climberFront.set(ControlMode.PercentOutput, 0);
        } else if(loc == BACK) {
            climberBack.set(ControlMode.PercentOutput, 0);
        }
    }

    public void driveForward() {
        climberWheels.set(defaultDriveSpeed);
    }

    public void driveReverse() {
        climberWheels.set(-defaultDriveSpeed);
    }

    public void stopDriving() {
        climberWheels.set(0);
    }

    public boolean isLegBelow(Location loc) {
        if (loc == FRONT) {
            if(frontHallEffectTop.get()) {
                foundFrontTop = true;
                return true;
            } else {
                return false;
            }
        } else {
            if (backHallEffectTop.get()) {
                foundBackTop = true; 
                return true;
            } else {
                return false;
            }
        }
    }

    public boolean isLegAbove(Location loc) {
        if (loc == FRONT) {
            if(frontHallEffectBottom.get()) {
                foundFrontBottom = true;
                return true;
            } else {
                return false;
            }
        } else {
            if (backHallEffectBottom.get()) {
                foundBackBottom = true; 
                return true;
            } else {
                return false;
            }
        }  
    }


    public enum Location {
        FRONT,
        BACK
    }

}