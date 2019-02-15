/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Stuff for climber
 */
public class Climber extends TalonSRX {
/**
     * The default speed to make controlling easier
     */
    private double defaultSpeed;
    private boolean found = false;

    private DigitalInput frontHallEffectTop = new DigitalInput(RobotMap.INPUT_FRONT_TOP_SW);
    private DigitalInput backHallEffectTop = new DigitalInput(RobotMap.INPUT_BACK_TOP_SW);
    private DigitalInput frontHallEffectBottom = new DigitalInput(RobotMap.INPUT_FRONT_BOTTOM_SW);
    private DigitalInput backHallEffectBottom = new DigitalInput(RobotMap.INPUT_BACK_BOTTOM_SW);

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
    Climber(int canPort, double defaultSpeed) {
        super(canPort); // Set up the motor controller
        this.defaultSpeed = defaultSpeed;
        this.name = String.format("Prototype_CAN (%d)", canPort);
    }

    public void forward() {
        if(frontHallEffectTop.get()) {

        }
        this.set(this.defaultSpeed);
    }

    public void reverse() {
        this.set(-this.defaultSpeed);
    }

    public void stop() {
        this.set(0.0);
    }

    
     // Set the speed of the motor controller
    public void set(double speed) {
        SmartDashboard.putNumber(name, speed); // for use in debugging
        super.set(ControlMode.PercentOutput, speed);
    }

    public boolean isFrontHEAtTop() {
        return frontHallEffectTop.get();
    }

    public boolean isFrontHEAtBottom() {
        return frontHallEffectBottom.get();
    }

    public boolean isBackHEAtTop() {
        return backHallEffectTop.get();
    }
    public boolean isBackHEAtBottom() {
        return backHallEffectBottom.get();
    }

}