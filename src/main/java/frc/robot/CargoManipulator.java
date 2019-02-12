package frc.robot;

public class CargoManipulator{
    static private boolean inShootingPosition;
    enum wheelState{IN, OUT, OFF};
    static private wheelState wheels;

    static void intakePosition(){
        inShootingPosition = false;
        //TO DO something that lowers wheel intake to intake posistion
    }

    static void shootingPosition(){
        inShootingPosition = true;
        //TO DO something that raises wheeled intake to shooting position
    }

    static void wheelsIn(){
        wheels = wheelState.IN;
        //TO DO something to turn the wheels to spinning in
    }

    static void wheelsShoot(){
        wheels = wheelState.OUT;
        //TO DO something that makes the wheels turn out
    }

    static void wheelsOff(){
        wheels = wheelState.OFF;
        //TO DO something that forces the wheels off
    }

    static boolean inShootingPosition(){
        return inShootingPosition;
    }

    static wheelState getWheelState(){
        return wheels;
    }


}