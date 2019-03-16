package frc.robot;

import java.awt.geom.Point2D;

public class Point {
     
    double x;
    double y;

    Point (double x, double y) {
        this.x = x;
        this.y = y;
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

    //not done
    double getAngle() {
        return 0;
    }
}