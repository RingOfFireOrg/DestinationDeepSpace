package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FrontTread
{
    private TalonSRX motorOne, motorTwo;

    public FrontTread(TalonSRX motorOne, TalonSRX motorTwo)
    {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
    }

    public void stop()
    {
        SmartDashboard.putString("Tread", "no");
        motorOne.set(ControlMode.PercentOutput, 0.0);
    }

    public void driveForward(double x)
    {
        SmartDashboard.putString("Tread", "yes");
        motorOne.set(ControlMode.PercentOutput, x);
        motorTwo.set(ControlMode.PercentOutput, x);
    }
}