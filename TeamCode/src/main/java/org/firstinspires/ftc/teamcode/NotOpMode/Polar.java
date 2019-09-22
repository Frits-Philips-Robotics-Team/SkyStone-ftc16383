package org.firstinspires.ftc.teamcode.NotOpMode;

public class Polar {

    private double theta;
    private double r;

    Polar(double theta, double r) {
        this.theta = theta;
        this.r = r;
    }

    public double getTheta() {
        return theta;
    }

    public double getDegrees() {
        return Math.toDegrees(theta);
    }

    static Polar fromXYCoord(double x, double y) {
        double r = Math.hypot(x, y); // length of vector from origin to point (x, y)
        double theta = Math.atan2(y, x); // Angle in RAD from positive X-axis to point (x, y)
        return new Polar(theta, r);
    }

    public double getX() {
        return r * Math.cos(theta);
    }

    public double getY() {
        return r * Math.sin(theta);
    }

}
