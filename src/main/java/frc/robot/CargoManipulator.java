package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoManipulator {
    enum intakePosition {
        INTAKE, LOWER_ROCKET, CARGO_SHIP, UP, ELSE
    };

    enum wheelState {
        IN, OUT, OFF
    };

    private intakePosition position;
    private wheelState wheels;
    public TalonSRX leftIntakeWheel;
    public TalonSRX rightIntakeWheel;
    public TalonSRX cargoArmMotor;
    private PID armAngleControl;
    private AnalogInput cargoEncoder;
    private double adjustment = 0;
    private boolean atTargetAngle = false;

    private static CargoManipulator cargoManipulator;

    final double INTAKE_POSITION_DEGREES = -6;
    final double LOWER_ROCKET_POSITION_DEGREES = 24;
    final double CARGO_SHIP_POSITION_DEGREES = 70;
    final double UP_POSITION_DEGREES = 90;
    double customTargetAngle = 0;

    protected CargoManipulator() {
        leftIntakeWheel = new TalonSRX(RobotMap.LEFT_INTAKE_WHEEL);
        rightIntakeWheel = new TalonSRX(RobotMap.RIGHT_INTAKE_WHEEL);
        cargoArmMotor = new TalonSRX(RobotMap.CARGO_ARM);
        wheels = wheelState.OFF;
        armAngleControl = new PID(0.0075, 0.00002, 0);
        armAngleControl.setOutputRange(-0.75, 0.75);
        cargoEncoder = new AnalogInput(RobotMap.ENCODER_CARGO_ARM);
    }

    // ensures we only ever have one instance of our manipulator
    public static CargoManipulator getInstance() {
        if (cargoManipulator == null) {
            cargoManipulator = new CargoManipulator();
        }
        return cargoManipulator;
    }

    // Helps for if the encoder slips we can just adjust all the values
    public void adjustAllTargets(double adjustment) {
        this.adjustment = adjustment;
    }

    // if for some reason we need to give it an angle other than a set scoring angle
    public void setToCustomPosition(double targetAngle) {
        this.position = intakePosition.ELSE;
        moveCargoArmToAngle(targetAngle);
        this.customTargetAngle = targetAngle;
    }

    public void setToIntakePosition() {
        // what is this even doing
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
        case CARGO_SHIP:
            setToCargoShipPosition();
            break;
        case UP:
            setToUpPosition();
            break;
        case ELSE:
            setToCustomPosition(customTargetAngle);
            break;
        }
    }

    public void moveArmUp(double speed) {
        cargoArmMotor.set(ControlMode.PercentOutput, speed);
    }

    public void moveArmDown(double speed) {
        cargoArmMotor.set(ControlMode.PercentOutput, -speed);
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
        double error = targetAngle + adjustment - currentAngle();
        armAngleControl.setError(error);
        armAngleControl.update();
        cargoArmMotor.set(ControlMode.PercentOutput, armAngleControl.getOutput());
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
        return (180.0 - (cargoEncoder.getVoltage() * 54.0));
        // this number is to convert voltage which comes back 0 to 5 to a number from 0
        // to 270 which is what the potentiometer returns. It is subtrated from 180 so
        // that zero is the intake arm being flat
    }

    public double currentAngle() {
        SmartDashboard.putNumber("CargoEncoder", getEncoderInDegrees());
        SmartDashboard.putNumber("Cargo Voltage", cargoEncoder.getVoltage());
        return (getEncoderInDegrees());
    }
}