package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;

public class Beak {
    private boolean isOpen;
    private boolean isExtended;
    private Servo beakActuator = new Servo(RobotMap.BEAK_ACTUATOR_CHANNEL);
    private VictorSP beakDeployer = new VictorSP(9);

    private static Beak beak;

    protected Beak(){
        beakActuator.set(0);
        beakDeployer.set(0);
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

    void extend() {
        beakDeployer.set(1);
        isExtended = true;
    }

    void retract() {
        beakDeployer.set(-1);
        isExtended = false;
    }

    void stopActuation() {
        beakDeployer.set(0);
    }

    boolean isOpen(){
        return isOpen;
    }    

    boolean isOut(){
        return isOut();
    }
}