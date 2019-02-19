package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import static frc.robot.Climber.Location.FRONT;
import static frc.robot.Climber.Location.BACK;

public class ClimberController {
    private Joystick climberStick = new Joystick(RobotMap.CLIMBER_JOYSTICK);

    private JoystickButton extendFrontBtn = new JoystickButton(climberStick, RobotMap.CLIMBER_EXTEND_FRONT);
    private JoystickButton extendBackBtn = new JoystickButton(climberStick, RobotMap.CLIMBER_EXTEND_BACK);
    private JoystickButton retractFrontBtn = new JoystickButton(climberStick, RobotMap.CLIMBER_RETRACT_FRONT);
    private JoystickButton retractBackBtn = new JoystickButton(climberStick, RobotMap.CLIMBER_RETRACT_BACK);
    private JoystickButton startAutoClimbBtn = new JoystickButton(climberStick, RobotMap.START_AUTOCLIMB);
    private JoystickButton stopAutoClimbBtn = new JoystickButton(climberStick, RobotMap.STOP_AUTOCLIMB);
    private Climber climber = new Climber();
    private AutoClimb autoClimb;

    public ClimberController(SwerveDrive swerveDrive) {
        autoClimb = new AutoClimb(climber, swerveDrive);
    }

    public void run() {
        climber.printHallEffectState();

        if (startAutoClimbBtn.get()) {
            autoClimb.autoClimbRestart();
        } else if (stopAutoClimbBtn.get()) {
            autoClimb.stopAutoClimb();
        }

        if (autoClimb.autoClimbEnabled()) {
            autoClimb.autoClimb();
        } else {
            climbManually();            
        }
    }

    private void climbManually() {
        double climberDrive = climberStick.getY();

        if (climberDrive > 0.25) {
            climber.driveForward();
        } else if (climberDrive < -0.25) {
            climber.driveReverse();
        } else {
            climber.stopDriving();
        }

        if (extendFrontBtn.get()) {
            climber.extendManual(FRONT);
        } else if (retractFrontBtn.get()) {
            climber.retractManual(FRONT);
        } else {
            climber.stopClimbing(FRONT);
        }

        if (extendBackBtn.get()) {
            climber.extendManual(BACK);
        } else if (retractBackBtn.get()) {
            climber.retractManual(BACK);
        } else {
            climber.stopClimbing(BACK);
        }
    }
}