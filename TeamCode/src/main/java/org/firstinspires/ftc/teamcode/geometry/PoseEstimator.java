package org.firstinspires.ftc.teamcode.geometry;

import androidx.annotation.NonNull;

import java.util.function.IntSupplier;

public class PoseEstimator {
    // Constants
    double trackWidth = 28.65;
    double strafeOffset = 5.75 * 2.54; //cm
    double ticksPerInch = 0.0013482;

    public double centimetersPerTick =  (3.2 * Math.PI) / 2000;

    final private IntSupplier leftOdo, rightOdo, backOdo;
    double deltaLeft, deltaRight, deltaBack;
    private double x = 0;
    private double y = 0;
    private double heading = 0;
    private int prevLeft = 0;
    private int prevRight = 0;
    private int prevBack = 0;

    public PoseEstimator(IntSupplier left, IntSupplier right, IntSupplier back) {
        this.leftOdo = left;
        this.rightOdo = right;
        this.backOdo = back;
    }

    public void resetHeading(double targetAngle)
    {
        heading -= heading;
        heading += Math.toRadians(targetAngle);
    }

    public double getHeading() {
//        double deltaTheta = (deltaRight - deltaLeft) / trackWidth;
//        heading += deltaTheta;
        return heading;
    }
    public double getGlobalX() {
        return x;
    }
    public double getGlobalY() {
        return y;
    }

    public void update() {
        // ticks
        int currentLeft = leftOdo.getAsInt();
        int currentRight = rightOdo.getAsInt();
        int currentBack = backOdo.getAsInt();

        deltaLeft = currentLeft - prevLeft;
        deltaRight = currentRight - prevRight;
        deltaBack = currentBack - prevBack;

        prevLeft = currentLeft;
        prevRight = currentRight;
        prevBack = currentBack;

        // cm
        double deltaLeftCentimeters = deltaLeft * centimetersPerTick;
        double deltaRightCentimeters = deltaRight * centimetersPerTick;
        double deltaBackCentimeters = deltaBack * centimetersPerTick;

        double deltaTheta = ((deltaRightCentimeters - deltaLeftCentimeters) / trackWidth); // tried deleting the divide by 4 to see if it would fix heading calculations

        heading += deltaTheta;

        double localDeltaY = (deltaLeftCentimeters + deltaRightCentimeters) / 2;
        double localDeltaX = deltaBackCentimeters - (deltaTheta * strafeOffset);
        double globalDeltaX = (localDeltaX * Math.cos(heading) - localDeltaY * Math.sin(heading));
        double globalDeltaY = localDeltaX * Math.sin(heading) + localDeltaY * Math.cos(heading);
        x += globalDeltaX;
        y += globalDeltaY;
    }

    @NonNull
    @Override
    public String toString()
    {
        return getGlobalX() + "X, " + getGlobalY() + "Y, " + getHeading() + "Theta";
    }
}