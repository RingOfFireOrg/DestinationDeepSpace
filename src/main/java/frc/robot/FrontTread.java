package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FrontTread {
    private SpeedControllerGroup motorGroup;

    public FrontTread(SpeedControllerGroup motorGroup) {
        this.motorGroup = motorGroup;
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