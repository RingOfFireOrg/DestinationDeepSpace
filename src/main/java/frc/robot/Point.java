package frc.robot;

public class Point {
     
    double x;
    double y;

    Point (double x, double y) {
        this.x = x;
        this.y = y;
    }

    Point () {
        this.x = 0;
        this.y = 0;
    }

    void setX(double x) {
        this.x = x;
    }

    void setY(double y) {
        this.y = y;
    }

    double distanceFromZero() {
        return Math.sqrt((x * x) + (y * y));
    }

    double distanceFromPoint(double a, double b) {
        return Math.sqrt(Math.pow(x - a, 2) + Math.pow(y - b, 2));
    }

    double getX() {
        return this.x;
    }

    double getY() {
        return this.y;
    }

    //not done <------------ <--------------- <-----------
    double getAngle() {
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

    double getCompassAngle() {
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