package org.firstinspires.ftc.teamcode.PurePursuit;

public class Vector2d {
    public double x;
    public double y;

    public static final Vector2d undefined = new Vector2d(Double.NaN, Double.NaN);

    // Constructors
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d() {
        this(0, 0);
    }

    // Polar constructor (magnitude, angle, isPolar flag)
    public Vector2d(double mag, double angle, boolean isPolar) {
        if (isPolar) {
            this.x = mag * Math.cos(angle);
            this.y = mag * Math.sin(angle);
        } else {
            this.x = mag;
            this.y = angle;
        }
    }

    // Get magnitude
    public double magnitude() {
        return Math.hypot(x, y);
    }

    // Get angle
    public double angle() {
        return Math.atan2(y, x);
    }

    // Distance to another point
    public double distanceTo(Vector2d other) {
        return Math.hypot(this.x - other.x, this.y - other.y);
    }

    // Rotate by angle (returns new vector)
    public Vector2d rotate(double angleRad) {
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);
        return new Vector2d(
                x * cos - y * sin,
                x * sin + y * cos
        );
    }

    // Add vectors
    public Vector2d add(Vector2d other) {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    // Subtract vectors
    public Vector2d subtract(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    // Scale vector
    public Vector2d scale(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Vector2d)) return false;
        Vector2d other = (Vector2d) obj;
        // Check for undefined vectors
        return (Double.isNaN(x) && Double.isNaN(other.x) &&
                Double.isNaN(y) && Double.isNaN(other.y)) ||
                (Math.abs(x - other.x) < 1e-6 && Math.abs(y - other.y) < 1e-6);
    }

    @Override
    public String toString() {
        return String.format("Vector2d(x=%.2f, y=%.2f)", x, y);
    }
}