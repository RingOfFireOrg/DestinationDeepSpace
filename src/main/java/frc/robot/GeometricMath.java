package frc.robot;

public class GeometricMath {

    static double limitRange(double input, double min, double max) {
        if (input < min) {
            input = min;
        } else if (input > max) {
            input = max;
        }
        return input;
    }
    
    static double pointDistance(Point a, Point b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    static Point vectorAddition(Point a, Point b) {
        return new Point(a.getX() + b.getX(), a.getY() + b.getY());
    }

    static Point vectorSubtraction(Point a, Point b) {
        return new Point(a.getX() - b.getX(), a.getY() - b.getY());
    }

    static Point midPoint(Point a, Point b) {
        return new Point((a.getX() + b.getX()) / 2, (a.getY() + b.getY()) / 2);
    }

    static Point rotateVector(Point a, double rotationInDegrees) {
        double magnitude = a.distanceFromZero();
        double initialAngle = Math.toDegrees(Math.atan(a.getY() / a.getX()));
        if (a.getX() < 0) {
            if (a.getY() < 0) {
                initialAngle -= 180;
            } else {
                initialAngle += 180;
            }
        }
        double finalAngle = initialAngle + rotationInDegrees;
        return new Point(magnitude * Math.cos(Math.toRadians(finalAngle)), magnitude * Math.sin(Math.toRadians(finalAngle)));
    }

}