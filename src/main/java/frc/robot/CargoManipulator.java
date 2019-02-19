package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoManipulator{
    enum intakePosition{INTAKE, LOWER_ROCKET, CARGO_SHIP, UP, STALL};
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
        intakeHeightPID = new PID(0.1, 0, 0);
        intakeHeightPID.setOutputRange(-1, 1);
        cargoEncoder = new AnalogInput(4);
        target = 0;
    }

    public static CargoManipulator getInstance(){
        if(cargoManipulator == null){
            cargoManipulator = new CargoManipulator();
        }
        return cargoManipulator;
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
       if (position == intakePosition.INTAKE) {
           target = 0;
           //intakeLift.set(ControlMode.PercentOutput, -0.5);
       } else if (position == intakePosition.LOWER_ROCKET) {
           target = 30;
       } else if (position == intakePosition.CARGO_SHIP) {
           target = 60;
       } else if (position == intakePosition.UP) {
           //intakeLift.set(ControlMode.PercentOutput, 0.5);
           target = 90;
       } else {
           //intakeLift.set(ControlMode.PercentOutput, 0);
       }

       intakeHeightPID.setError(target - currentAngle());
       intakeHeightPID.update();
       intakeLift.set(ControlMode.PercentOutput, intakeHeightPID.getOutput());


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
   }

    intakePosition getPosition(){
        return position;
    }

    wheelState getWheelState(){
        return wheels;
    }

    private double currentAngle() {
        SmartDashboard.putNumber("CargoEncoder", 180 - (cargoEncoder.getVoltage() * 54));
        return 180 - (cargoEncoder.getVoltage() * 54);
    }


}