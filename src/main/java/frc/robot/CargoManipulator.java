package frc.robot;

public class CargoManipulator{
    private boolean inShootingPosition;
    private boolean wheelsAreOn;
    private boolean wheelsSpinningIn;

    void intakePosition(){
        inShootingPosition = false;
        //TO DO something that lowers wheel intake to intake posistion
    }

    void shootingPosition(){
        inShootingPosition = true;
        //TO DO something that raises wheeled intake to shooting position
    }

    void wheelsIn(){
        wheelsAreOn = true;
        wheelsSpinningIn = true;
        //TO DO something to turn the wheels to spinning in
    }

    void wheelsShoot(){
        wheelsAreOn = true;
        wheelsSpinningIn = false;
        //TO DO something that makes the wheels turn out
    }

    void wheelsOff(){
        wheelsAreOn = false;
        //TO DO something that forces the wheels off
    }

    boolean inShootingPosition(){
        return inShootingPosition;
    }

    boolean wheelsAreOn(){
        return wheelsAreOn;
    }

    boolean wheelsSpinningIn(){
        return wheelsSpinningIn;
    }
}