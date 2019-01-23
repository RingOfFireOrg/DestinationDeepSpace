package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This allows us to easily add prototypes to the robot
 */
public class Prototype_PWM extends TalonSRX {
    /**
     * The default speed to make controlling easier
     */
    private double defaultSpeed;

    /**
     * The name of the object (for use in debug)
     */
    private String name;

    /**
     * The constructor for our class
     * 
     * @param canPort      - which port on the roboRio it is connectedTo
     * @param defaultSpeed - what speed should be used for forward and reverse
     */
    Prototype_PWM(int canPort, double defaultSpeed) {
        super(canPort); // Set up the motor controller
        this.defaultSpeed = defaultSpeed;
        this.name = String.format("Prototype_PWM (%d)", canPort);
    }

    /**
     * Move prototype motor forward
     */
    public void forward() {
        this.set(this.defaultSpeed);
    }

    /**
     * Move prototype motor in reverse
     */
    public void reverse() {
        this.set(-this.defaultSpeed);
    }

    /**
     * Stop prototype motor
     */
    public void stop() {
        this.set(0.0);
    }

    /**
     * Set the speed of the motor controller
     */
    public void set(double speed) {
        SmartDashboard.putNumber(name, speed); // for use in debugging
        super.set(ControlMode.PercentOutput, speed);
    }

}