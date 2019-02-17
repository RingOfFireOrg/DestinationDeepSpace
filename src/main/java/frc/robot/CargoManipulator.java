package frc.robot;

public class CargoManipulator{
    enum intakePosition{INTAKE, LOWER_ROCKET, CARGO_SHIP};
    private intakePosition position;
    enum wheelState{IN, OUT, OFF};
    private wheelState wheels;

    public CargoManipulator(){ 
        wheels = wheelState.OFF;
    }
    
    void intakePosition(){
        position = intakePosition.INTAKE;
        //TO DO something that lowers wheel intake to intake posistion
    }

    void lowerRocketPosition(){
        position = intakePosition.LOWER_ROCKET;
        //TO DO something that raises wheeled intake to shooting position
    }

    void cargoShipPosition() {
        position = intakePosition.CARGO_SHIP;
    }

    void wheelsIn(){
        wheels = wheelState.IN;
        //TO DO something to turn the wheels to spinning in
    }

    void wheelsShoot(){
        wheels = wheelState.OUT;
        //TO DO something that makes the wheels turn out
    }

    void wheelsOff(){
        wheels = wheelState.OFF;
        //TO DO something that forces the wheels off
    }

    intakePosition getPosition(){
        return position;
    }

    wheelState getWheelState(){
        return wheels;
    }


}