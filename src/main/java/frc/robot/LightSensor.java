package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LightSensor extends Object {

    private DigitalInput sensorA;
    private DigitalInput sensorB;
    private String name;

    LightSensor(int portA, int portB, String sensorName) {
        super();
        sensorA = new DigitalInput(portA);
        sensorB = new DigitalInput(portB);
        name = sensorName;
    }

    Boolean get() {
        SmartDashboard.putBoolean("Light_" + name + "_LA", sensorA.get());
        SmartDashboard.putBoolean("Light_" + name + "_LB", sensorB.get());

        return sensorA.get();
    }

}