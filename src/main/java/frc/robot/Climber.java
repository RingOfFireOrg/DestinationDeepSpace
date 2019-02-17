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

    private MovementState frontState = MovementState.ALLOW;
    private MovementState backState = MovementState.ALLOW;

    private TalonSRX climberFront = new TalonSRX(RobotMap.CAN_CLIMBER_FRONT);
    private TalonSRX climberBack = new TalonSRX(RobotMap.CAN_CLIMBER_BACK);
    private TalonSRX climberLeftWheel = new TalonSRX(RobotMap.CAN_CLIMBER_WHEEL_LEFT);
    private TalonSRX climberRightWheel = new TalonSRX(RobotMap.CAN_CLIMBER_WHEEL_RIGHT);

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
        updateLegState(Direction.UP);
        climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.UP, frontState));
        climberBack.set(ControlMode.PercentOutput, getDriveSpeed(Direction.UP, backState));
    }

    public void down() {
        updateLegState(Direction.DOWN);
        climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.DOWN, frontState));
        climberBack.set(ControlMode.PercentOutput, getDriveSpeed(Direction.DOWN, backState));
    }

    public static double getDriveSpeed(Direction direction, MovementState current) {
        if (direction == Direction.UP) {
            switch (current) {
            case STOP_UP:
            case STOP_MAGNET_UP:
                return RobotMap.SPEED_STOP;
            case SLOW_UP:
            case SLOW_MAGNET_UP:
                return RobotMap.SPEED_SLOW_CLIMB;
            default:
                return RobotMap.SPEED_DEFAULT_CLIMB;
            }
        } else {
            switch (current) {
            case STOP_DOWN:
            case STOP_MAGNET_DOWN:
                return RobotMap.SPEED_STOP;
            case SLOW_DOWN:
            case SLOW_MAGNET_DOWN:
                return -RobotMap.SPEED_SLOW_CLIMB;
            default:
                return -RobotMap.SPEED_DEFAULT_CLIMB;
            }
        }
    }

    // public void up(Location loc) {
    // if (loc == FRONT && foundFrontTop != MovementState.STOP) {
    // climberFront.set(ControlMode.PercentOutput, -defaultClimbSpeed);
    // } else if (loc == BACK && !foundBackTop) {
    // climberBack.set(ControlMode.PercentOutput, -defaultClimbSpeed);
    // }
    // }

    // public void down(Location loc) {
    // if (loc == BACK && !foundBackBottom) {
    // climberBack.set(ControlMode.PercentOutput, defaultClimbSpeed);
    // } else if (loc == FRONT && !foundFrontBottom) {
    // climberFront.set(ControlMode.PercentOutput, defaultClimbSpeed);
    // }
    // }

    public void stopClimbing() {
        stopClimbing(FRONT);
        stopClimbing(BACK);
    }

    public void stopClimbing(Location loc) {
        if (loc == FRONT) {
            climberFront.set(ControlMode.PercentOutput, 0);
        } else if (loc == BACK) {
            climberBack.set(ControlMode.PercentOutput, 0);
        }
    }

    public void driveForward() {
        climberLeftWheel.set(ControlMode.PercentOutput, defaultDriveSpeed);
        climberRightWheel.set(ControlMode.PercentOutput, defaultDriveSpeed);
    }

    public void driveReverse() {
        climberLeftWheel.set(ControlMode.PercentOutput, -defaultDriveSpeed);
        climberRightWheel.set(ControlMode.PercentOutput, -defaultDriveSpeed);
    }

    public void stopDriving() {
        climberLeftWheel.set(ControlMode.PercentOutput, 0);
        climberRightWheel.set(ControlMode.PercentOutput, 0);
    }

    public void updateLegState(Direction direction) {
        frontState = getState(frontState, frontHallEffectTop.get(), direction);
        backState = getState(backState, backHallEffectTop.get(), direction);
        SmartDashboard.putString("Front state", frontState.toString());
        SmartDashboard.putString("Back state", backState.toString());
    }

    public static MovementState getState(MovementState current, boolean sensorDetected, Direction direction) {

        if (current == MovementState.STOP_UP) {
            if (direction == Direction.DOWN && sensorDetected) {
                return MovementState.STOP_MAGNET_UP;
            }
        }

        if (current == MovementState.STOP_MAGNET_UP) {
            if (direction == Direction.UP && !sensorDetected) {
                return MovementState.STOP_UP;
            }
            if (direction == Direction.DOWN && !sensorDetected) {
                return MovementState.SLOW_UP;
            }
        }

        if (current == MovementState.SLOW_UP) {
            if (direction == Direction.UP && sensorDetected) {
                return MovementState.STOP_MAGNET_UP;
            }
            if (direction == Direction.DOWN && sensorDetected) {
                return MovementState.SLOW_MAGNET_UP;
            }
        }

        if (current == MovementState.SLOW_MAGNET_UP) {
            if (direction == Direction.UP && !sensorDetected) {
                return MovementState.SLOW_UP;
            }
            if (direction == Direction.DOWN && !sensorDetected) {
                return MovementState.ALLOW;
            }
        }

        if (current == MovementState.ALLOW) {
            if (direction == Direction.UP && sensorDetected) {
                return MovementState.SLOW_MAGNET_UP;
            }
            if (direction == Direction.DOWN && sensorDetected) {
                return MovementState.SLOW_MAGNET_DOWN;
            }
        }

        if (current == MovementState.SLOW_MAGNET_DOWN) {
            if (direction == Direction.UP && !sensorDetected) {
                return MovementState.ALLOW;
            }
            if (direction == Direction.DOWN && !sensorDetected) {
                return MovementState.SLOW_DOWN;
            }
        }

        if (current == MovementState.SLOW_DOWN) {
            if (direction == Direction.UP && sensorDetected) {
                return MovementState.SLOW_MAGNET_DOWN;
            }
            if (direction == Direction.DOWN && sensorDetected) {
                return MovementState.STOP_MAGNET_DOWN;
            }
        }

        if (current == MovementState.STOP_MAGNET_DOWN) {
            if (direction == Direction.UP && !sensorDetected) {
                return MovementState.SLOW_DOWN;
            }
            if (direction == Direction.DOWN && !sensorDetected) {
                return MovementState.STOP_DOWN;
            }
        }

        if (current == MovementState.STOP_DOWN) {
            if (direction == Direction.UP && sensorDetected) {
                return MovementState.STOP_MAGNET_DOWN;
            }
        }

        return current;
    }

    public enum Location {
        FRONT, BACK
    }

    public enum Direction {
        UP, DOWN
    }

}