package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class PotentiometerEncoder extends AnalogInput {

    protected double angleOffset;

    public PotentiometerEncoder (int channel) {
        super(channel);
        this.angleOffset = 0;
    }
    
    public PotentiometerEncoder (int channel, double angleOffset) {
        super(channel);
        this.angleOffset = angleOffset;
    }

    public void setOffsetAngle (double angleOffset) {
        this.angleOffset = angleOffset;
    }

    public double getAngle() {
        double angle = angleOffset - (getVoltage() * 54);
        return angle;
    }
}