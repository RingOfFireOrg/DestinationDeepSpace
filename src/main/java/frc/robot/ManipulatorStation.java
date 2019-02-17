package frc.robot;

public class ManipulatorStation{
    
    Vision limelight = new Vision();

    public ManipulatorStation(){

    }

    public void runManipulation(){
        //this if else statement MUST be the first part of this class so that if you are running automation nothing else can occur
        if((limelight.hatchPickup() || limelight.hatchScore() || limelight.cargoScoreCargoShip() || limelight.cargoScoreRocket()) && limelight.automationRunning){
            return;
        } else {
            limelight.automationRunning = false;
        }


        //everything else goes below

    }






}