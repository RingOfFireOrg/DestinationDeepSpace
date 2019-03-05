package frc.robot;

import static frc.robot.Climber.Location.BACK;
import static frc.robot.Climber.Location.FRONT;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;


public class ClimberController {
    private XboxController manipulatorController = new XboxController(RobotMap.MANIPULATOR_GAMEPAD);
    private Joystick climberStick = new Joystick(RobotMap.CLIMBER_JOYSTICK);
    private boolean extendFrontBtn = manipulatorController.getBumperPressed(GenericHID.Hand.kRight);
    private boolean extendBackBtn = manipulatorController.getBumperPressed(GenericHID.Hand.kLeft);
    private double retractFrontBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kRight);
    private double retractBackBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kLeft);
    private JoystickButton startAutoClimbBtn = new JoystickButton(climberStick, RobotMap.START_AUTOCLIMB);
    private JoystickButton stopAutoClimbBtn = new JoystickButton(climberStick, RobotMap.STOP_AUTOCLIMB);
    private Climber climber = new Climber();
    private AutoClimb autoClimb;

    public ClimberController(SwerveDrive swerveDrive, AHRS ahrs) {
        autoClimb = new AutoClimb(climber, swerveDrive, ahrs);
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
        double climberDrive = manipulatorController.getRawAxis(1);

        if (climberDrive > 0.25) {
            climber.driveForward();
        } else if (climberDrive < -0.25) {
            climber.driveReverse();
        } else {
            climber.stopDriving();
        }

        if (extendFrontBtn) {
            climber.extendManual(FRONT);
        } else if (retractFrontBtn > 0.5) {
            climber.retractManual(FRONT);
        } else {
            climber.stopClimbing(FRONT);
        }

        if (extendBackBtn) {
            climber.extendManual(BACK);
        } else if (retractBackBtn > 0.5) {
            climber.retractManual(BACK);
        } else {
            climber.stopClimbing(BACK);
        }
    }
}