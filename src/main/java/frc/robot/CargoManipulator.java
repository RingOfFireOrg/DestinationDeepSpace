package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoManipulator{
    enum intakePosition{INTAKE, LOWER_ROCKET, CARGO_SHIP, UP, STALL, ELSE};
    private intakePosition position;
    enum wheelState{IN, OUT, OFF};
    private wheelState wheels;
    private double wheelPower = RobotMap.WHEEL_INTAKE_SPEED;
    public TalonSRX leftIntakeWheel;
    public TalonSRX rightIntakeWheel;
    public TalonSRX intakeLift;
    private PID intakeHeightPID;
    private AnalogInput cargoEncoder;
    private double target;

    private static CargoManipulator cargoManipulator;

    protected CargoManipulator(){ 
        leftIntakeWheel = new TalonSRX(RobotMap.LEFT_INTAKE_WHEEL);
        rightIntakeWheel = new TalonSRX(RobotMap.RIGHT_INTAKE_WHEEL);
        intakeLift = new TalonSRX(RobotMap.CARGO_ARM);
        wheels = wheelState.OFF;
        position = intakePosition.STALL;
        intakeHeightPID = new PID(0.0075, 0.00002, 0);
        intakeHeightPID.setOutputRange(-0.75, 0.75);
        cargoEncoder = new AnalogInput(4);
        target = 0;
    }

    public static CargoManipulator getInstance(){
        if(cargoManipulator == null){
            cargoManipulator = new CargoManipulator();
        }
        return cargoManipulator;
    }

    public void overrideTarget(double adjustment) {
        target += adjustment;
    }

    public void setIntake() {
        this.position = intakePosition.INTAKE;
    }

    public void setLowerRocket() {
        this.position = intakePosition.LOWER_ROCKET;
    }

    public void setCargoShip() {
        this.position = intakePosition.CARGO_SHIP;
    }

    public void setUp() {
        this.position = intakePosition.UP;
    }

    public void setStall() {
        this.position = intakePosition.STALL;
    }

    public void setOff() {
        this.wheels = wheelState.OFF;
    }

    public void setIn() {
        this.wheels = wheelState.IN;
    }

    public void setOut() {
        this.wheels = wheelState.OUT;
    }
    
   public void updateCargo() {
       if (wheels == wheelState.IN) {
           leftIntakeWheel.set(ControlMode.PercentOutput, wheelPower);
           rightIntakeWheel.set(ControlMode.PercentOutput, -wheelPower);
       } else if (wheels == wheelState.OUT) {
           leftIntakeWheel.set(ControlMode.PercentOutput, -1);
           rightIntakeWheel.set(ControlMode.PercentOutput, 1);
       } else {
           leftIntakeWheel.set(ControlMode.PercentOutput, 0);
           rightIntakeWheel.set(ControlMode.PercentOutput, 0);
       }

       if (position == intakePosition.INTAKE) {
           if (currentAngle() < 0) {
               intakeLift.set(ControlMode.PercentOutput, 0);
               return;
           }
           target = -6;
       } else if (position == intakePosition.LOWER_ROCKET) {
           target = 24;
       } else if (position == intakePosition.CARGO_SHIP) {
           target = 70;
       } else if (position == intakePosition.UP) {
           target = 90;
       } else if (position == intakePosition.ELSE) {
    
       }

       intakeHeightPID.setError(target - currentAngle());
       intakeHeightPID.update();
       intakeLift.set(ControlMode.PercentOutput, intakeHeightPID.getOutput());
   }

    intakePosition getPosition(){
        return position;
    }

    wheelState getWheelState(){
        return wheels;
    }

    public double currentAngle() {
        SmartDashboard.putNumber("CargoEncoder", 180.0 - (cargoEncoder.getVoltage() * 54.0));
        SmartDashboard.putNumber("Voltage", cargoEncoder.getVoltage());
        return (180.0 - (cargoEncoder.getVoltage() * 54.0));
    }


}