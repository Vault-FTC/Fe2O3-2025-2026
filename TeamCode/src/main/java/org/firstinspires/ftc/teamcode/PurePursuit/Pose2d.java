package org.firstinspires.ftc.teamcode.PurePursuit;

public class Pose2d {
    private Vector2d position;
    private Rotation2d heading;

    public Pose2d(Vector2d position, Rotation2d heading) {
        this.position = position;
        this.heading = heading;
    }

    public Pose2d() {
        this(new Vector2d(), new Rotation2d());
    }

    public Vector2d getPosition() {
        return position;
    }

    public double getX() {
        return position.x;
    }

    public double getY() {
        return position.y;
    }

    public Rotation2d getHeading() {
        return heading;
    }

    public Pose2d copy() {
        return new Pose2d(
                new Vector2d(position.x, position.y),
                new Rotation2d(heading.getAngleRadians())
        );
    }
}
