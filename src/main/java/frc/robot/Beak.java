package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Beak {
    private boolean isOpen;
    private Servo beakActuator; //= new Servo(RobotMap.BEAK_ACTUATOR_CHANNEL);
    private static Beak beak;

    protected Beak() {
        beakActuator = new Servo(RobotMap.BEAK_ACTUATOR_CHANNEL);
        // Alan - this needs setBounds or no good....
        beakActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        isOpen = false;
    }

    public static Beak getInstance() {
        if (beak == null) {
            beak = new Beak();
        }
        return beak;
    }

    void open() {
        beakActuator.setSpeed(1.0);
        isOpen = true;
    }

    void close(){
        beakActuator.setSpeed(-1.0);
        isOpen = false;
    }

    boolean isOpen(){
        return isOpen;
    }    
}