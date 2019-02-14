package frc.robot;

public class CargoManipulator{
    private boolean inShootingPosition;
    enum wheelState{IN, OUT, OFF};
    private wheelState wheels;

    public CargoManipulator(){

    }
    
    void intakePosition(){
        inShootingPosition = false;
        //TO DO something that lowers wheel intake to intake posistion
    }

    void shootingPosition(){
        inShootingPosition = true;
        //TO DO something that raises wheeled intake to shooting position
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

    boolean inShootingPosition(){
        return inShootingPosition;
    }

    wheelState getWheelState(){
        return wheels;
    }


}