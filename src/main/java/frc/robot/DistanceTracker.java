package frc.robot;

public class DistanceTracker {
    double startPoint = 0;

    //excludes the back left encoder atm

    DistanceTracker (double startPoint) {
        this.startPoint = startPoint;
    }

    DistanceTracker () {
        this.startPoint = this.getDistance();
    }

    public void reset() {
        startPoint = this.getDistance();
    }
    public double getDistance() {
        return 0.0; //needs real number here
    }
    
}