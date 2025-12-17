package org.firstinspires.ftc.teamcode.geometry;

import org.firstinspires.ftc.teamcode.PurePursuit.Pose2d;

public class RobotVelocity2D {
    TimeStamp timeStamp = new TimeStamp();
    Pose2d lastPose; // Think of this as a timestamp to compare previous times

    Vector2D translationVel; // m / s
    double angularVel; // Rad / s

    public RobotVelocity2D(Pose2d currentPose){
        this.lastPose = currentPose;
        timeStamp.reset();
    }

    public RobotVelocity2D(Pose2d lastPose, Vector2D translationVel, double angularVel){
        this.lastPose = lastPose.copy();
        this.translationVel = translationVel.copy();
        this.angularVel = angularVel;
    }

    public double getAngularVel(){
        return angularVel;
    }

    public Vector2D getTranslationVel(){
        return translationVel.copy();
    }

    public void setAngularVel(double angularVel){
        this.angularVel = angularVel;
    }

    public void setTranslationVel(Vector2D translationVel){
        this.translationVel = translationVel;
    }

    /**
     * NOTE: If the robot moves more than 180 deg between runs of this function, the calculation will likely be wrong.
     * Call this in periodic, or not at all.
     */
    public void updateVel(Pose2d currentPose){
        double dt = timeStamp.getDt();
        double x_change = currentPose.getX() - lastPose.getX();
        double y_change = currentPose.getY() - lastPose.getY();
        translationVel = new Vector2D(x_change / dt, y_change / dt);

        // angles kinda suck so this is the best way to represent the cutoffs from 2PI to 0.
        double rot_change =
                (currentPose.getHeading().getAngleRadians() - lastPose.getHeading().getAngleRadians()
                + Math.PI) % (2 * Math.PI) - Math.PI;
        angularVel = (rot_change < -Math.PI ? rot_change + 2 * Math.PI : rot_change) / dt;
    }

    public RobotVelocity2D copy(){
        return new RobotVelocity2D(lastPose, translationVel, angularVel);
    }
}
