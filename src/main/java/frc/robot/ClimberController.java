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
    public Joystick leftDriveJoystick = new Joystick(RobotMap.LEFT_DRIVE_JOYSTICK);
    private JoystickButton startAutoClimbGamepadBtn = new JoystickButton(manipulatorController, RobotMap.MANIPULATOR_START_BUTTON_VALUE);
    private JoystickButton stopAutoClimbGamepadBtn = new JoystickButton(manipulatorController, RobotMap.MANIPULATOR_BACK_BUTTON_VALUE);
    private JoystickButton startAutoClimbBtn = new JoystickButton(manipulatorPanel, RobotMap.OPEN_BEAK_BUTTON);
    private JoystickButton startAutoClimbBtn2 = new JoystickButton(manipulatorPanel, RobotMap.CLOSE_BEAK_BUTTON); //should find better names for these 2 buttons
    private JoystickButton climbModeToggle = new JoystickButton(manipulatorPanel, RobotMap.CLIMBING_MODE_PROTECTED_SWITCH);
    private JoystickButton enableClimbButton = new JoystickButton(leftDriveJoystick, RobotMap.CLIMBER_ENABLE_BUTTON);
    private Climber climber = new Climber();
    private AutoClimb autoClimb;

    private boolean climbModeTrue;


    public ClimberController(SwerveDrive swerveDrive, AHRS ahrs, CargoManipulator cargoManipulator) {
        autoClimb = new AutoClimb(climber, swerveDrive, ahrs, cargoManipulator);
    }

    private boolean startAutoClimbTrue(){ //should find a better name
        if (climbModeTrue && startAutoClimbBtn.get() && startAutoClimbBtn2.get()){
            return true;
        } else if (climbModeTrue && startAutoClimbGamepadBtn.get()) {
            return true;
        } else {
            return false;
        }
        
    }

    public void run() {
        climber.printHallEffectState();

        climbModeTrue = climbModeToggle.get() || enableClimbButton.get();

        if (startAutoClimbTrue()) {
            autoClimb.autoClimbRestart();
        } else if (!climbModeTrue || stopAutoClimbGamepadBtn.get()) {
            autoClimb.stopAutoClimb();
        }

        if (autoClimb.autoClimbEnabled()) {
            autoClimb.autoClimb();
        } else {
            climbManually();            
        }
    }

    private void climbManually() {
        double climberDrive = manipulatorController.getRawAxis(RobotMap.MANIPULATOR_LEFT_STICK_Y_AXIS);
        boolean extendFrontBtn = manipulatorController.getBumper(GenericHID.Hand.kRight);
        boolean extendBackBtn = manipulatorController.getBumper(GenericHID.Hand.kLeft);
        double retractFrontBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kRight);
        double retractBackBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kLeft);
        
        if (!climbModeTrue){
            return;
        }
        
        if (climberDrive > 0.25) {
            climber.driveReverse();
        } else if (climberDrive < -0.25) {
            climber.driveForward();
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