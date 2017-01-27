package org.firstinspires.ftc.teamcode;

/**
 * Created by Gabe on 12/18/2016.
 */

public class robotUtils {

    private double rotateRadius;
    private double wheelCircumference;
    private double wheelRadius;

    public robotUtils(double wheelSeperation, double wheelDiameter){
        wheelRadius = wheelDiameter / 2;
        wheelCircumference = wheelDiameter * Math.PI;
        rotateRadius = wheelSeperation / 2;
    }

    public int degreesToTicks(double degreesToRotate){
        double radiansToRotate = Math.toRadians(degreesToRotate);
        double rotateArc = radiansToRotate * rotateRadius;
        double revolutionCounts = rotateArc / wheelCircumference;
        int ticks = (int) revolutionCounts * 1440;
        return ticks;

    }

    public int inchesToTicks(double distance){
        double rotations = distance / wheelCircumference;
        int ticks = (int) rotations * 1440;
        return ticks;
    }

    public int degreesToTicksSingleMotor(double degreesToRotate){
        double radiansToRotate = Math.toRadians(degreesToRotate);
        double rotateArc = radiansToRotate * rotateRadius;
        double revolutionCounts = rotateArc / wheelCircumference;
        int ticks = (int) revolutionCounts * 1440;
        return ticks;

    }
}
