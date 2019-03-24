package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoManipulator {
    enum intakePosition {
        INTAKE, LOWER_ROCKET, MID_ROCKET, CARGO_SHIP, UP, ELSE, MANUAL_MODE
    };

    enum wheelState {
        IN, OUT, OFF
    };

    enum encoderPresence {
        LEFT, RIGHT, BOTH, NONE
    }

    private intakePosition position = intakePosition.MANUAL_MODE;
    private wheelState wheels;
    private encoderPresence currentEncoderPresence;
    public TalonSRX leftIntakeWheel;
    public TalonSRX rightIntakeWheel;
    public TalonSRX cargoArmMotor;
    private PID armAngleControl;
    private PotentiometerEncoder rightCargoEncoder;
    private PotentiometerEncoder leftCargoEncoder;
    private boolean atTargetAngle = false;

    private static CargoManipulator cargoManipulator;

    // relative to level
    final double INTAKE_POSITION_DEGREES = -7;
    final double LOWER_ROCKET_POSITION_DEGREES = 23; // 26
    final double MID_ROCKET_POSITION_DEGREES = 50; // 78
    final double CARGO_SHIP_POSITION_DEGREES = 43; // 65
    final double UP_POSITION_DEGREES = 90;
    double customTargetAngle = 0;

    // level Angle
    private final double ZERO_DEGREE_ARM_VALUE = -10;

    protected CargoManipulator() {
        leftIntakeWheel = new TalonSRX(RobotMap.LEFT_INTAKE_WHEEL);
        rightIntakeWheel = new TalonSRX(RobotMap.RIGHT_INTAKE_WHEEL);
        cargoArmMotor = new TalonSRX(RobotMap.CARGO_ARM);
        wheels = wheelState.OFF;
        currentEncoderPresence = encoderPresence.RIGHT;
        armAngleControl = new PID(0.0075, 0.00002, 0);
        armAngleControl.setOutputRange(-0.75, 0.75);
        rightCargoEncoder = new PotentiometerEncoder(RobotMap.RIGHT_ENCODER_CARGO_ARM, 180);
        leftCargoEncoder = new PotentiometerEncoder(RobotMap.LEFT_ENCODER_CARGO_ARM, 90);
    }

    // ensures we only ever have one instance of our manipulator
    public static CargoManipulator getInstance() {
        if (cargoManipulator == null) {
            cargoManipulator = new CargoManipulator();
        }
        return cargoManipulator;
    }

    // if for some reason we need to give it an angle other than a set scoring angle
    public void setToCustomPosition(double targetAngle) {
        this.position = intakePosition.ELSE;
        moveCargoArmToAngle(targetAngle);
        this.customTargetAngle = targetAngle;
    }

    public void setToIntakePosition() {
        // what is this even doing?
        if (currentAngle() < 0) {
            cargoArmMotor.set(ControlMode.PercentOutput, 0);
            return;
        } // else:
        moveCargoArmToAngle(INTAKE_POSITION_DEGREES);
        this.position = intakePosition.INTAKE;
    }

    public void setToLowerRocketPosition() {
        moveCargoArmToAngle(LOWER_ROCKET_POSITION_DEGREES);
        this.position = intakePosition.LOWER_ROCKET;
    }

    public void setToMidRocketPosition() {
        moveCargoArmToAngle(MID_ROCKET_POSITION_DEGREES);
        this.position = intakePosition.MID_ROCKET;
    }

    public void setToCargoShipPosition() {
        moveCargoArmToAngle(CARGO_SHIP_POSITION_DEGREES);
        this.position = intakePosition.CARGO_SHIP;
    }

    public void setToUpPosition() {
        moveCargoArmToAngle(UP_POSITION_DEGREES);
        this.position = intakePosition.UP;
    }

    public void setToCurrentPosition() {
        switch (position) {
        case INTAKE:
            setToIntakePosition();
            break;
        case LOWER_ROCKET:
            setToLowerRocketPosition();
            break;
        case MID_ROCKET:
            setToMidRocketPosition();
            break;
        case CARGO_SHIP:
            setToCargoShipPosition();
            break;
        case UP:
            setToUpPosition();
            break;
        case MANUAL_MODE:
            stopArm();
            break;
        case ELSE:
            setToCustomPosition(customTargetAngle);
            break;
        }
    }

    public void moveArmUp(double speed) {
        cargoArmMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("CargoArmPower", speed);
        this.position = intakePosition.MANUAL_MODE;
    }

    public void moveArmDown(double speed) {
        cargoArmMotor.set(ControlMode.PercentOutput, -speed);
        SmartDashboard.putNumber("CargoArmPower", -speed);
        this.position = intakePosition.MANUAL_MODE;
    }

    public void moveArm(double speed) {
        cargoArmMotor.set(ControlMode.PercentOutput, speed);
        this.position = intakePosition.MANUAL_MODE;
    }

    public void stopArm() {
        cargoArmMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setWheelsOff() {
        this.wheels = wheelState.OFF;
        leftIntakeWheel.set(ControlMode.PercentOutput, RobotMap.LEFT_CARGO_WHEEL_OFF_SPEED);
        rightIntakeWheel.set(ControlMode.PercentOutput, RobotMap.RIGHT_CARGO_WHEEL_OFF_SPEED);
    }

    public void setWheelsIn() {
        this.wheels = wheelState.IN;
        leftIntakeWheel.set(ControlMode.PercentOutput, RobotMap.LEFT_CARGO_WHEEL_INTAKE_SPEED);
        rightIntakeWheel.set(ControlMode.PercentOutput, RobotMap.RIGHT_CARGO_WHEEL_INTAKE_SPEED);
    }

    public void setWheelsOut() {
        this.wheels = wheelState.OUT;
        leftIntakeWheel.set(ControlMode.PercentOutput, RobotMap.LEFT_CARGO_WHEEL_SHOOT_SPEED);
        rightIntakeWheel.set(ControlMode.PercentOutput, RobotMap.RIGHT_CARGO_WHEEL_SHOOT_SPEED);
    }

    private void moveCargoArmToAngle(double targetAngle) {
        double error = targetAngle + currentAngle();
        if (currentAngle() == 270) {
            cargoArmMotor.set(ControlMode.PercentOutput, 0);
            return;
        }
        armAngleControl.setError(error);
        armAngleControl.update();
        cargoArmMotor.set(ControlMode.PercentOutput, armAngleControl.getOutput() + 0.2);
        if (Math.abs(error) < 5) {
            atTargetAngle = true;
        } else {
            atTargetAngle = false;
        }
    }

    intakePosition getPosition() {
        return position;
    }

    boolean getAtTargetAngle() {
        return atTargetAngle;
    }

    wheelState getWheelState() {
        return wheels;
    }

    double getEncoderInDegrees() {
        return -(180.0 - (leftCargoEncoder.getVoltage() * 54.0)); // remove once following code has been reviewed

        // insert this once it has been reviewed <-------- :0 <------- :) <--------- !!!
        // if (rightCargoEncoder.getVoltage() < 4.8 && rightCargoEncoder.getVoltage() >
        // 0.2) {
        // return (180.0 - (rightCargoEncoder.getVoltage() * 54.0));
        // } else if (leftCargoEncoder.getVoltage() < 4.8 &&
        // leftCargoEncoder.getVoltage() > 0.2) {
        // return 270 - (180.0 - (leftCargoEncoder.getVoltage() * 54.0)); //temporary
        // value -- get actual offset and such later
        // } else {
        // return 270;
        // }

        // switch (currentEncoderPresence) {
        // case BOTH:
        // if (rightCargoEncoder.getAngle() < 105 && rightCargoEncoder.getAngle() > -15
        // && -leftCargoEncoder.getAngle() < 105 && -leftCargoEncoder.getAngle() > -15)
        // {
        // return (rightCargoEncoder.getAngle() - leftCargoEncoder.getAngle()) / 2;
        // } else if (rightCargoEncoder.getAngle() < 105 && rightCargoEncoder.getAngle()
        // > -15) {
        // return rightCargoEncoder.getAngle();
        // } else if (leftCargoEncoder.getAngle() < 105 && leftCargoEncoder.getAngle() >
        // -15) {
        // return -leftCargoEncoder.getAngle();
        // } else {
        // return RobotMap.FAILURE_RETURN_ENCODER_VALUE;
        // }
        // case LEFT:
        // if (-leftCargoEncoder.getAngle() < 105 && -leftCargoEncoder.getAngle() > -15)
        // {
        // return -leftCargoEncoder.getAngle();
        // }
        // return RobotMap.FAILURE_RETURN_ENCODER_VALUE;
        // case RIGHT:
        // if (rightCargoEncoder.getAngle() > 105 && rightCargoEncoder.getAngle() > -15)
        // {
        // return rightCargoEncoder.getAngle();
        // }
        // return RobotMap.FAILURE_RETURN_ENCODER_VALUE;
        // case NONE:
        // return RobotMap.FAILURE_RETURN_ENCODER_VALUE;
        // default:
        // return RobotMap.FAILURE_RETURN_ENCODER_VALUE;
        // }
    }

    public double currentAngle() {
        SmartDashboard.putNumber("CargoEncoder", ZERO_DEGREE_ARM_VALUE - getEncoderInDegrees());
        SmartDashboard.putNumber("Raw Cargo Encoder Degrees", getEncoderInDegrees());
        SmartDashboard.putNumber("Cargo Voltage Right: ", rightCargoEncoder.getVoltage());
        SmartDashboard.putNumber("Cargo Voltage Left: ", leftCargoEncoder.getVoltage());
        return (ZERO_DEGREE_ARM_VALUE - getEncoderInDegrees());
    }
}