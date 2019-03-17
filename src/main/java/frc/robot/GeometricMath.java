package frc.robot;

public class GeometricMath {

    double pointDistance(Point a, Point b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    Point addVectors(Point a, Point b) {
        return new Point(a.getX() + b.getX(), a.getY() + b.getY());
    }

}