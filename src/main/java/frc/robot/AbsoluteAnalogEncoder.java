package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class AbsoluteAnalogEncoder extends AnalogInput {
	public AbsoluteAnalogEncoder(int channel) {
		super(channel);
	}
	
	public double getAngle(){
		double angle = (getVoltage() / 5.0) * 360.0;
		return angle;
	}

}
