package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;

public class Beak {
    private boolean isOpen;
    private Servo beakActuator = new Servo(RobotMap.BEAK_ACTUATOR_CHANNEL);

    private static Beak beak;

    protected Beak(){
        beakActuator.set(0);
    }
    
    public static Beak getInstance(){
        if(beak == null){
            beak = new Beak();
        }
        return beak;
    }

    void open(){
        beakActuator.set(0);
        isOpen = true;
    }

    void close(){
        beakActuator.set(1);
        isOpen = false;
    }

    boolean isOpen(){
        return isOpen;
    }    
}