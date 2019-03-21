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

public class Climber {
    private MovementState frontState = MovementState.STOP_MAGNET_DOWN;
    private MovementState backState = MovementState.STOP_MAGNET_DOWN;

    private TalonSRX climberFront = new TalonSRX(RobotMap.CAN_CLIMBER_FRONT);
    private VictorSP climberBack = new VictorSP(RobotMap.CAN_CLIMBER_BACK);
    private VictorSP climberWheels = new VictorSP(RobotMap.CAN_CLIMBER_WHEELS);

    private DigitalInput frontHallEffect = new DigitalInput(RobotMap.INPUT_FRONT_SW);
    private DigitalInput backHallEffect = new DigitalInput(RobotMap.INPUT_BACK_SW);

    private static boolean doStopAtL2 = false; 

    public void reset() {
        frontState = MovementState.STOP_MAGNET_DOWN;
        backState = MovementState.STOP_MAGNET_DOWN;
    }

    //extends both bars
    public void extend() {
        updateLegState(Direction.EXTEND);
        climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.EXTEND, frontState));
        climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE * getDriveSpeed(Direction.EXTEND, backState));
        doStopAtL2 = false;
    }

    //extends one bar depending on a passed in enum
    public void extend(Location location) {
        updateLegState(Direction.EXTEND);

        if (location == Location.FRONT) {
            climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.EXTEND, frontState));
        } else {
            climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE*getDriveSpeed(Direction.EXTEND, backState));
        }
    }

    //retracts both bars
    public void retract() {
        updateLegState(Direction.RETRACT);
        climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.RETRACT, frontState));
        climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE*getDriveSpeed(Direction.RETRACT, backState));
    }

    //retracts one bar 
    public void retract(Location location) {
        updateLegState(Direction.RETRACT);

        if (location == Location.FRONT) {
            climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.RETRACT, frontState));
        } else {
            climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE*getDriveSpeed(Direction.RETRACT, backState));
        }
    }

    //takes robot to the top, handles whether to stop cause of magnet and also regulating the two bars to keep them at the same height
    public void extendLevel(double difference) {
        SmartDashboard.putNumber("difference", difference);
        updateLegState(Direction.EXTEND);
        //assuming that the gyro will be + when rotated forward
        if (getDriveSpeed(Direction.EXTEND, frontState) == RobotMap.SPEED_DEFAULT_CLIMB) {
            climberFront.set(ControlMode.PercentOutput, RobotMap.SPEED_DEFAULT_CLIMB + difference);
        } else {
            climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.EXTEND, frontState));
        }
 
        if (getDriveSpeed(Direction.EXTEND, backState) == RobotMap.SPEED_DEFAULT_CLIMB) {
            climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE * (RobotMap.SPEED_DEFAULT_CLIMB - difference));
        } else {
            climberBack.set(getDriveSpeed(Direction.EXTEND, backState));
        }
    }

        //takes robot to the top, handles whether to stop cause of magnet and also regulating the two bars to keep them at the same height
        public void extendLevelManual(double difference) {
            SmartDashboard.putNumber("difference", difference);
            //assuming that the gyro will be + when rotated forward
            climberFront.set(ControlMode.PercentOutput, RobotMap.SPEED_DEFAULT_CLIMB + difference);
            climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE * (RobotMap.SPEED_DEFAULT_CLIMB - difference));
        }

    //not used, not needed
    //takes robot to the bottom
    public void retractLevel(double difference) {
        updateLegState(Direction.RETRACT);
        if (getDriveSpeed(Direction.RETRACT, frontState) == -RobotMap.SPEED_DEFAULT_CLIMB) {
            climberFront.set(ControlMode.PercentOutput, -RobotMap.SPEED_DEFAULT_CLIMB - difference);
        } else {
            climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.RETRACT, frontState));
        }

        if (getDriveSpeed(Direction.RETRACT, backState) == -RobotMap.SPEED_DEFAULT_CLIMB) {
            climberBack.set(-RobotMap.SPEED_DEFAULT_CLIMB + difference);
        } else {
            climberBack.set(getDriveSpeed(Direction.RETRACT, backState));
        }
    }

    public void extendManual(Location location) {
        if (location == Location.FRONT) {
            climberFront.set(ControlMode.PercentOutput, RobotMap.SPEED_DEFAULT_CLIMB);
        } else {  //if(location == Location.BACK)
            climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE * RobotMap.SPEED_DEFAULT_CLIMB);
        }
    }

    public void retractManual(Location location) {
        if (location == Location.FRONT) {
            climberFront.set(ControlMode.PercentOutput, -RobotMap.SPEED_DEFAULT_CLIMB);
        } else {
            climberBack.set(RobotMap.BACK_CLIMBER_SPEED_MULTIPLE * -RobotMap.SPEED_DEFAULT_CLIMB);
        }
    }

    public void retractManual(Location location, double speed) {
        if (location == Location.FRONT) {
            if (getDriveSpeed(Direction.RETRACT, frontState) == -RobotMap.SPEED_DEFAULT_CLIMB) {
                climberFront.set(ControlMode.PercentOutput, -RobotMap.SPEED_DEFAULT_CLIMB * speed);
            } else {
                climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.RETRACT, frontState));
            }
        } else {
            if (getDriveSpeed(Direction.RETRACT, backState) == -RobotMap.SPEED_DEFAULT_CLIMB) {
                climberFront.set(ControlMode.PercentOutput, -RobotMap.SPEED_DEFAULT_CLIMB * speed);
            } else {
                climberFront.set(ControlMode.PercentOutput, getDriveSpeed(Direction.RETRACT, backState));
            }
        }
    }

    public boolean isFullyExtendedL3() {
        return isFullyExtendedL3(FRONT) && isFullyExtendedL3(BACK);
    }

    public boolean isFullyExtendedL3(Location location) {
        if (Location.FRONT == location) {
            return isFullyExtendedL3(frontState);
        } else {
            return isFullyExtendedL3(backState);
        }
    }

    public static boolean isFullyExtendedL3(MovementState current) {
        return current == MovementState.STOP_UP || current == MovementState.STOP_MAGNET_UP;
    }

    public boolean isFullyExtendedL2() {
        return isFullyExtendedL2(FRONT) && isFullyExtendedL2(BACK);
    }

    public boolean isFullyExtendedL2(Location location) {
        if (Location.FRONT == location) {
            return isFullyExtendedL2(frontState);
        } else {
            return isFullyExtendedL2(backState);
        }
    }

    public static boolean isFullyExtendedL2(MovementState current) {
        return current == MovementState.LEVEL_2_MAGNET;
    }

    public void setToExtendL2() {
        doStopAtL2 = true;
    }

    public boolean isFullyRetracted() {
        return isFullyRetracted(FRONT) && isFullyRetracted(BACK);
    }

    public boolean isFullyRetracted(Location location) {
        if (Location.FRONT == location) {
            return isFullyRetracted(frontState);
        } else {
            return isFullyRetracted(backState);
        }
    }

    public static boolean isFullyRetracted(MovementState current) {
        return current == MovementState.STOP_DOWN || current == MovementState.STOP_MAGNET_DOWN;
    }

   //actually returning a goal NOT an actual speed - let's eventually change this method name
    public static double getDriveSpeed(Direction direction, MovementState current) {
        if (direction == Direction.EXTEND) {
            switch (current) {
            case STOP_UP:
            case STOP_MAGNET_UP:
                return RobotMap.SPEED_STOP;
            case SLOW_UP:
            case SLOW_MAGNET_UP:
                return RobotMap.SPEED_SLOW_CLIMB;
            case ALLOW_ABOVE_L2:
            case LEVEL_2_MAGNET:
                if (doStopAtL2) {
                    return RobotMap.SPEED_STOP;
                }
            default:
                return RobotMap.SPEED_DEFAULT_CLIMB;
            }
        } else { //if(direction == Direction.RETRACT)
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

    public void stopClimbing() {
        stopClimbing(FRONT);
        stopClimbing(BACK);
    }

    public void stopClimbing(Location loc) {
        if (loc == FRONT) {
            climberFront.set(ControlMode.PercentOutput, 0);
        } else if (loc == BACK) {
            climberBack.set(0);
        }
    }

    public void driveForward() {
        climberWheels.set(RobotMap.SPEED_DEFAULT_DRIVE);
    }

    public void driveReverse() {
        climberWheels.set(-RobotMap.SPEED_DEFAULT_DRIVE);
    }

    public void stopDriving() {
        climberWheels.set(0);
    }

    public void updateLegState(Direction direction) {
        frontState = getState(frontState, !frontHallEffect.get(), direction);
        backState = getState(backState, !backHallEffect.get(), direction);
        SmartDashboard.putString("Front state", frontState.toString());
        SmartDashboard.putString("Back state", backState.toString());
    }

    public static MovementState getState(MovementState current, boolean sensorDetected, Direction direction) {

        if (current == MovementState.STOP_UP) {
            if (direction == Direction.RETRACT && sensorDetected) {
                return MovementState.STOP_MAGNET_UP;
            }
        }

        if (current == MovementState.STOP_MAGNET_UP) {
            if (direction == Direction.EXTEND && !sensorDetected) {
                return MovementState.STOP_UP;
            }
            if (direction == Direction.RETRACT && !sensorDetected) {
                return MovementState.SLOW_UP;
            }
        }

        if (current == MovementState.SLOW_UP) {
            if (direction == Direction.EXTEND && sensorDetected) {
                return MovementState.STOP_MAGNET_UP;
            }
            if (direction == Direction.RETRACT && sensorDetected) {
                return MovementState.SLOW_MAGNET_UP;
            }
        }

        if (current == MovementState.SLOW_MAGNET_UP) {
            if (direction == Direction.EXTEND && !sensorDetected) {
                return MovementState.SLOW_UP;
            }
            if (direction == Direction.RETRACT && !sensorDetected) {
                return MovementState.ALLOW_ABOVE_L2;
            }
        }

        if (current == MovementState.ALLOW_ABOVE_L2) {
            if (direction == Direction.EXTEND && sensorDetected) {
                return MovementState.SLOW_MAGNET_UP;
            }
            if (direction == Direction.RETRACT && sensorDetected) {
                return MovementState.LEVEL_2_MAGNET;
            }
        }

        if(current == MovementState.LEVEL_2_MAGNET) {
            if (direction == Direction.EXTEND && !sensorDetected) {
                return MovementState.ALLOW_ABOVE_L2;
            } 
            if (direction == Direction.RETRACT && !sensorDetected) {
                return MovementState.ALLOW_BELOW_L2;
            }
        }

        if(current == MovementState.ALLOW_BELOW_L2) {
            if (direction == Direction.EXTEND && sensorDetected) {
                return MovementState.LEVEL_2_MAGNET;
            }
            if (direction == Direction.RETRACT && sensorDetected) {
                return MovementState.SLOW_MAGNET_DOWN;
            }
        }

        if (current == MovementState.SLOW_MAGNET_DOWN) {
            if (direction == Direction.EXTEND && !sensorDetected) {
                return MovementState.ALLOW_BELOW_L2;
            }
            if (direction == Direction.RETRACT && !sensorDetected) {
                return MovementState.SLOW_DOWN;
            }
        }

        if (current == MovementState.SLOW_DOWN) {
            if (direction == Direction.EXTEND && sensorDetected) {
                return MovementState.SLOW_MAGNET_DOWN;
            }
            if (direction == Direction.RETRACT && sensorDetected) {
                return MovementState.STOP_MAGNET_DOWN;
            }
        }

        if (current == MovementState.STOP_MAGNET_DOWN) {
            if (direction == Direction.EXTEND && !sensorDetected) {
                return MovementState.SLOW_DOWN;
            }
            if (direction == Direction.RETRACT && !sensorDetected) {
                return MovementState.STOP_DOWN;
            }
        }

        if (current == MovementState.STOP_DOWN) {
            if (direction == Direction.EXTEND && sensorDetected) {
                return MovementState.STOP_MAGNET_DOWN;
            }
        }

        return current;
    }

    public void printHallEffectState(){
        SmartDashboard.putBoolean("Front HE Sensor", !frontHallEffect.get());
        SmartDashboard.putBoolean("Back HE sensor", !backHallEffect.get());
    }

    public boolean isClimberDown() {
        return !backHallEffect.get() && !frontHallEffect.get();
    }

    public enum Location {
        FRONT, BACK
    }

    public enum Direction {
        EXTEND, RETRACT
    }
}