
package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Pneumatics extends Subsystem {
  Solenoid Left1;
  Solenoid Right1;
  public Pneumatics(Solenoid solenoid1, Solenoid solenoid2) {
    Left1 = solenoid1;
    Right1 = solenoid2;
  }


  @Override
  public void initDefaultCommand() {
    
  }

  public void pistonOut() {

  }

}
