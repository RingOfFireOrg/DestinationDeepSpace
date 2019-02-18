package frc.robot;

public class ManipulatorStation{
    
    Vision limelight = new Vision();

    public ManipulatorStation(){

    }

    public void runManipulation(){
        //this if statement MUST be the first part of this class so that if you are running automation nothing else can occur
        if(limelight.isAutomationRunning()){ //when rug gets around to it there would be one like this coming from climber
            return;
        }


        //everything else goes below

    }

    






}