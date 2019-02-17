package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CargoManipulator{
    enum intakePosition{INTAKE, LOWER_ROCKET, CARGO_SHIP, UP};
    private intakePosition position;
    enum wheelState{IN, OUT, OFF};
    private wheelState wheels;
    private double wheelPower = RobotMap.WHEEL_INTAKE_SPEED;
    public TalonSRX leftIntakeWheel = new TalonSRX(RobotMap.LEFT_INTAKE_WHEEL);
    public TalonSRX rightIntakeWheel = new TalonSRX(RobotMap.RIGHT_INTAKE_WHEEL);
    private PID intakeHeightPID;

    public CargoManipulator(){ 
        wheels = wheelState.OFF;
        intakeHeightPID = new PID(0, 0, 0);
    }

    public void setPosition(intakePosition position) {
        this.position = position;
    }

    public void setState(wheelState wheels) {
        this.wheels = wheels;
    }
    
   public void updateManipulator() {
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