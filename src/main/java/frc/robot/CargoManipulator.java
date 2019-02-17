package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.VictorSP;

public class CargoManipulator{
    enum intakePosition{INTAKE, LOWER_ROCKET, CARGO_SHIP, UP};
    private intakePosition position;
    enum wheelState{IN, OUT, OFF};
    private wheelState wheels;
    private double wheelPower = RobotMap.WHEEL_INTAKE_SPEED;
    public TalonSRX leftIntakeWheel;
    public TalonSRX rightIntakeWheel;
    public VictorSP intakeLift;
    private PID intakeHeightPID;

    public CargoManipulator(){ 
        leftIntakeWheel = new TalonSRX(RobotMap.LEFT_INTAKE_WHEEL);
        rightIntakeWheel = new TalonSRX(RobotMap.RIGHT_INTAKE_WHEEL);
        intakeLift = new VictorSP(1);
        wheels = wheelState.OFF;
        position = intakePosition.UP;
        intakeHeightPID = new PID(0.1, 0, 0);
        intakeHeightPID.setOutputRange(-1, 1);
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
       double target;
       if (position == intakePosition.INTAKE) {
           target = 0;
       } else if (position == intakePosition.LOWER_ROCKET) {
           target = 30;
       } else if (position == intakePosition.CARGO_SHIP) {
           target = 60;
       } else {
           target = 90;
       }

       intakeHeightPID.setError(target - currentAngle());
       intakeHeightPID.update();
       intakeLift.set(intakeHeightPID.getOutput());


       if (wheels == wheelState.IN) {
           leftIntakeWheel.set(ControlMode.PercentOutput, wheelPower);
           rightIntakeWheel.set(ControlMode.PercentOutput, -wheelPower);
       } else if (wheels == wheelState.OUT) {
           leftIntakeWheel.set(ControlMode.PercentOutput, -wheelPower);
           rightIntakeWheel.set(ControlMode.PercentOutput, wheelPower);
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
        return 0;   
    }


}