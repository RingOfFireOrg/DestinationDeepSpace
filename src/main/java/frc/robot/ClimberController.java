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
    public Joystick manipulatorPanel = new Joystick(RobotMap.MANIPULATOR_PANEL);
    private JoystickButton startAutoClimbBtn = new JoystickButton(manipulatorPanel, RobotMap.OPEN_BEAK_BUTTON);
    private JoystickButton startAutoClimbBtn2 = new JoystickButton(manipulatorPanel, RobotMap.CLOSE_BEAK_BUTTON); //should find better names for these 2 buttons
    private JoystickButton climbModeToggle = new JoystickButton(manipulatorPanel, RobotMap.CLIMBING_MODE_PROTECTED_SWITCH);
    private Climber climber = new Climber();
    private AutoClimb autoClimb;

    public ClimberController(SwerveDrive swerveDrive, AHRS ahrs) {
        autoClimb = new AutoClimb(climber, swerveDrive, ahrs);
    }

    private boolean startAutoClimbTrue(){ //should find a better name
        if (startAutoClimbBtn.get() && startAutoClimbBtn2.get()){
            return false;
        } else {
            return false;
        }
        
    }

    boolean climbModeTrue = climbModeToggle.get();
    public void run() {
        climber.printHallEffectState();

        if (startAutoClimbTrue() && climbModeTrue) {
            autoClimb.autoClimbRestart();
        } else if (climbModeTrue == false) {
            autoClimb.stopAutoClimb();
        }

        if (autoClimb.autoClimbEnabled()) {
            autoClimb.autoClimb();
        } else {
            climbManually();            
        }
    }

    private void climbManually() {
        double climberDrive = manipulatorController.getRawAxis(RobotMap.RIGHT_STICK_Y_AXIS);
        boolean extendFrontBtn = manipulatorController.getBumper(GenericHID.Hand.kRight);
        boolean extendBackBtn = manipulatorController.getBumper(GenericHID.Hand.kLeft);
        double retractFrontBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kRight);
        double retractBackBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kLeft);
        
        
        if (climberDrive > 0.25 && climbModeTrue) {
            climber.driveReverse();
        } else if (climberDrive < -0.25 && climbModeTrue) {
            climber.driveForward();
        } else {
            climber.stopDriving();
        }

        if (extendFrontBtn && climbModeTrue) {
            climber.extendManual(FRONT);
        } else if (retractFrontBtn > 0.5 && climbModeTrue) {
            climber.retractManual(FRONT);
        } else {
            climber.stopClimbing(FRONT);
        }

        if (extendBackBtn && climbModeTrue) {
            climber.extendManual(BACK);
        } else if (retractBackBtn > 0.5 && climbModeTrue) {
            climber.retractManual(BACK);
        } else {
            climber.stopClimbing(BACK);
        }
    } 
}