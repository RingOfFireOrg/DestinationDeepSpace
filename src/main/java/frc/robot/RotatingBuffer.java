package frc.robot;

public class RotatingBuffer {
    
    double[] data;
    int length;
    int position = 0;

    RotatingBuffer(int length) {
        this.length = length;
        data = new double[length];
        for (int i = 0 ; i < length ; i ++) {
            data[i] = 0;
        }
    }

    RotatingBuffer(int length, int initialSetPoint) {
        this.length = length;
        data = new double[length];
        for (int i = 0 ; i < length ; i ++) {
            data[i] = initialSetPoint;
        }
    }

    public void add(double newData) {
        data[position] = newData;
        position ++;
        if (position >= length) {
            position = 0;
        }
    }

    public void setAll(double newData) {
        for (int i = 0 ; i < length ; i ++) {
            data[i] = newData;
        }
    }

    public double getAverage() {
        double result = 0;
        for (int i = 0 ; i < length ; i ++) {
            result += data[i];
        }
        return result / length;
    }

}