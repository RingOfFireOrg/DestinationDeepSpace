package frc.robot;

import static frc.robot.Climber.Location.BACK;
import static frc.robot.Climber.Location.FRONT;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private AHRS ahrs;

    private PID robotPitchPID;
    private double pitchOffset;

    private double climbAngle = 0;

    public ClimberController(SwerveDrive swerveDrive, AHRS ahrs, CargoManipulator cargoManipulator) {
        autoClimb = new AutoClimb(climber, swerveDrive, ahrs, cargoManipulator);
        this.ahrs = ahrs;
        robotPitchPID = new PID(0.02, 0.00005, 0);
        robotPitchPID.setOutputRange(-1, 1);
        pitchOffset = ahrs.getPitch();
        robotPitchPID.reset();
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
        SmartDashboard.putNumber("gyro pitch: ", ahrs.getPitch());

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
        double extendFrontBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kRight);
        double extendBackBtn = manipulatorController.getTriggerAxis(GenericHID.Hand.kLeft);
        boolean retractFrontBtn = manipulatorController.getBumper(GenericHID.Hand.kRight);
        boolean retractBackBtn = manipulatorController.getBumper(GenericHID.Hand.kLeft);
        
        if (!climbModeTrue){
            return;
        }

        // if (climber.isClimberDown()) {
        //     robotPitchPID.reset();
        // }

        if (extendFrontBtn > 0.5 && extendBackBtn > 0.5) {
            robotPitchPID.setError((pitchOffset - ahrs.getPitch()) - climbAngle);
            robotPitchPID.update(); 
            climber.extendLevelManual(robotPitchPID.getOutput());
        } else { 
            if (extendFrontBtn > 0.5) {
                climber.extendManual(FRONT);
            } else if (retractFrontBtn) {
                climber.retractManual(FRONT);
            } else {
                climber.stopClimbing(FRONT);
            }
         
            if(extendBackBtn > 0.5) {
                climber.extendManual(BACK);
            } else if (retractBackBtn) {
                climber.retractManual(BACK);
            } else {
                climber.stopClimbing(BACK);
            }
        }
       
        
        if (climberDrive > 0.25) {
            climber.driveReverse();
        } else if (climberDrive < -0.25) {
            climber.driveForward();
        } else {
            climber.stopDriving();
        }
    } 
}