package frc.robot.Utility;

public class Point {
     
    double x;
    double y;

    public Point (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point () {
        this.x = 0;
        this.y = 0;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double distanceFromZero() {
        return Math.sqrt((x * x) + (y * y));
    }

    public double distanceFromPoint(double a, double b) {
        return Math.sqrt(Math.pow(x - a, 2) + Math.pow(y - b, 2));
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    //not done <------------ <--------------- <-----------
    public double getAngle() {
        double angle = Math.toDegrees(Math.atan(x / y));
        if (x >= 0) {
            if (y >= 0) {
                // already in Q1
            } else {
                // shift to Q4
                angle += 180;
            }
        } else {
            if (y >= 0) {
                // shift to Q2
            } else {
                // shift to Q3
                angle -= 180;
            }
        }
        return angle;
    }

    public double getCompassAngle() {
        double angle = getAngle();
        while (angle <= -180 || angle > 180) {
            if (angle <= -180) {
                angle += 360;
            } else {
                angle -= 360;
            }
        }
        return angle;
    }
}