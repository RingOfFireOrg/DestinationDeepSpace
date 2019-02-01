package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FrontTread {
    private SpeedControllerGroup motorGroup;

    public FrontTread(WPI_TalonSRX motorOne, WPI_TalonSRX motorTwo) {
        this.motorGroup = new SpeedControllerGroup(motorOne, motorTwo);
    }

    public void stop() {
        SmartDashboard.putString("Tread", "no");
        this.motorGroup.stopMotor();
    }

    public void set(double speed) {
        SmartDashboard.putString("Tread", "yes");
        this.motorGroup.set(speed);
    }
}