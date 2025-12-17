package org.firstinspires.ftc.teamcode.PurePursuit;

public class Rotation2d {
    private double angleRad;

    public Rotation2d(double angleRad) {
        this.angleRad = angleRad;
    }

    public Rotation2d() {
        this(0);
    }

    public double getAngleRadians() {
        return angleRad;
    }

    public double getAngleDegrees() {
        return Math.toDegrees(angleRad);
    }

    public void setAngleRadians(double angleRad) {
        this.angleRad = angleRad;
    }

    public void setAngleDegrees(double angleDeg) {
        this.angleRad = Math.toRadians(angleDeg);
    }

    // Get shortest angular distance between two angles
    public static double getError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        // Normalize to [-PI, PI]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }

    // Rotate by another rotation
    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(this.angleRad + other.angleRad);
    }

    // Get cosine
    public double getCos() {
        return Math.cos(angleRad);
    }

    // Get sine
    public double getSin() {
        return Math.sin(angleRad);
    }

    public static String stringOf(Rotation2d rotation) {
        if (rotation == null) return "null";
        return String.format("%.2f°", rotation.getAngleDegrees());
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(%.2f° / %.3f rad)", getAngleDegrees(), angleRad);
    }
}