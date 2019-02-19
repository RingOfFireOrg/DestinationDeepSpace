package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;

public class Beak {
    private boolean isOpen;
    private Servo beakActuator = new Servo(RobotMap.BEAK_ACTUATOR_CHANNEL);

    private static Beak beak;

    protected Beak() {
    }

    public static Beak getInstance() {
        if (beak == null) {
            beak = new Beak();
        }
        return beak;
    }

    void open() {
        beakActuator.set(1);
        DriverStation.reportError("Beak Open", false);
        isOpen = true;
    }

    void close(){
        beakActuator.set(0);
        DriverStation.reportError("Beak Close", false);
        isOpen = false;
    }

    boolean isOpen(){
        return isOpen;
    }    
}