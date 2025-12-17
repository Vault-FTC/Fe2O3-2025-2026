package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.PurePursuit.PIDController;
import org.firstinspires.ftc.teamcode.PurePursuit.Path;
import org.firstinspires.ftc.teamcode.PurePursuit.Rotation2d;
import org.firstinspires.ftc.teamcode.PurePursuit.Vector2d;
import org.firstinspires.ftc.teamcode.PurePursuit.Waypoint;
import org.firstinspires.ftc.teamcode.PurePursuit.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.PoseEstimator;

import java.util.function.IntSupplier;

public class driveallclass extends Subsystem {
    private DcMotorEx frmotor, flmotor, brmotor, blmotor;
    IntSupplier rightOdo, leftOdo, backOdo;
    PoseEstimator poseEstimator;

    public int waypointIndex = 0;

    private Path followPath;
    private final ElapsedTime timer = new ElapsedTime();

    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = 0;
    private double followStartTimestamp;
    private Waypoint[][] segments;

    private final PIDController driveController =
            new PIDController(0.05, 0, 0);

    private final PIDController rotController =
            new PIDController(2.0, 0, 0);

    private double lastTargetAngle = 0;

    public enum DriveState {
        IDLE,
        FOLLOWING,
    }

    public DriveState driveState = DriveState.IDLE;

    public driveallclass(HardwareMap hardwareMap) {
        frmotor = hardwareMap.get(DcMotorEx.class, "rf");
        flmotor = hardwareMap.get(DcMotorEx.class, "lf");
        brmotor = hardwareMap.get(DcMotorEx.class, "rb");
        blmotor = hardwareMap.get(DcMotorEx.class, "lb");

        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// Reverse one side of motors if needed (depends on robot configuration)
        frmotor.setDirection(DcMotorEx.Direction.REVERSE);
        brmotor.setDirection(DcMotorEx.Direction.REVERSE);
        flmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftOdo = () -> brmotor.getCurrentPosition(); // left
        backOdo = () -> -blmotor.getCurrentPosition(); // back
        rightOdo = () -> frmotor.getCurrentPosition(); // right

        poseEstimator = new PoseEstimator(leftOdo, rightOdo, backOdo);
        poseEstimator.update();
        poseEstimator.resetHeading(0);
    }

    public void resetHeading(double heading) {
        poseEstimator.resetHeading(heading);
    }

    public String getPosition() {
        return "X: " + poseEstimator.getGlobalX() + " Y: " + poseEstimator.getGlobalY() + " Heading: " + Math.toDegrees(poseEstimator.getHeading());
    }

    public void drive(double forward, double right, double rotate) {
        double botHeading = poseEstimator.getHeading();

        // Rotate the movement vector by the inverse of the robot's heading
        // X is positive to the right, Y is positive up
        double rotatedX = forward * Math.sin(botHeading) - right * Math.cos(botHeading);
        double rotatedY = forward * Math.cos(botHeading) + right * Math.sin(botHeading);

        // Calculate motor powers
        double frontLeftPower = rotatedY + rotatedX + rotate;
        double backLeftPower = rotatedY - rotatedX + rotate;
        double frontRightPower = rotatedY - rotatedX - rotate;
        double backRightPower = rotatedY + rotatedX - rotate;
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frmotor.setPower(frontRightPower);
        brmotor.setPower(backRightPower);
        flmotor.setPower(frontLeftPower);
        blmotor.setPower(backLeftPower);

        poseEstimator.update();
    }

//    public void driveToPosition(Location target, double turnVal)
//    {
//        double p = 0.0001;
//        double strafe = (target.Strafe + (backOdo.getAsInt() * poseEstimator.centimetersPerTick));
//        double forward1 = (target.Forward - (leftOdo.getAsInt() * poseEstimator.centimetersPerTick));
//        double forward2 = (target.Forward - (rightOdo.getAsInt() * poseEstimator.centimetersPerTick));
//
//        // double strafe = (target.Strafe
//        double forward = (forward1 + forward2) / 2;
//        if(Math.abs(forward) <  400)
//        {
//            forward = 0;
//        }
//        if(Math.abs(strafe) < 400)
//        {
//            strafe = 0;
//        }
//        drive(forward * p, strafe * p, turnVal);
//    }

    public void driveToPosition(Location target, double turnVal, Telemetry telemetry) {
        double p = 0.008;
        double p_rotation = 0.03;
        double strafe = (target.Strafe - poseEstimator.getGlobalX());
        double forward = (-target.Forward + poseEstimator.getGlobalY());
            double heading = (target.TurnDegrees - Math.toDegrees(poseEstimator.getHeading()));

        double strafeError = (target.Strafe - poseEstimator.getGlobalX());
        double forwardError = (-target.Forward + poseEstimator.getGlobalY());
        double headingError = (target.TurnDegrees - Math.toDegrees(poseEstimator.getHeading()));


        if (telemetry != null) {
            telemetry.addData("Target", "X: " + target.Strafe + "  Y: " + target.Forward);
//            telemetry.addData("Target Strafe", strafe);
//            telemetry.addData("Target Forward", forward);
//            telemetry.addData("Target Heading", heading);
        }

        double forwardPower = forward * p;
        double strafePower = strafe * p;
        double turnPower = heading * p_rotation;

        double minPower = 0.25;
        double maxPower = 1;

        forwardPower = Math.max(Math.min(forwardPower, maxPower), -maxPower);
        strafePower = Math.max(Math.min(strafePower, maxPower), -maxPower);

        if (Math.abs(forwardPower) < minPower && Math.abs(forwardError) > 3) {
            forwardPower = minPower * Math.signum(forwardError);
        }
        if (Math.abs(strafePower) < minPower && Math.abs(strafeError) > 3) {
            strafePower = minPower * Math.signum(strafeError);
        }

        double distance = Math.hypot(forward,strafe);

        if (distance < 7) {
            drive(0,0,0);
            return;
        }

        drive(forwardPower, strafePower, turnPower);

    }

    public boolean isAtPosition(Location target) {
        double currentX = poseEstimator.getGlobalX();
        double currentY = poseEstimator.getGlobalY();
        double currentHeading = poseEstimator.getHeading();
        double tolerance = 15;
        double turnTolerance = 17.5;
        return Math.abs(currentX - target.Strafe) < tolerance &&
                Math.abs(currentY - target.Forward) < tolerance &&
                Math.abs(Math.toDegrees(currentHeading)-target.TurnDegrees) < turnTolerance;
    }

    private Pose2d getCurrentPose() {
        return new Pose2d(
                new Vector2d(
                        poseEstimator.getGlobalX(),
                        poseEstimator.getGlobalY()
                ),
                new Rotation2d(poseEstimator.getHeading())
        );
    }

    /**
     * Start following a path
     */
    public void setFollowPath(Path path) {
        waypointIndex = 0;
        driveState = DriveState.IDLE;
        followPath = path;
        segments = followPath.generateLineSegments();
        followStartTimestamp = timer.milliseconds();
    }

    /**
     * Drive to a specific waypoint with PID control
     */
    private void driveToWaypoint(Waypoint targetPoint, boolean useEndpointHeading) {
        Pose2d botPose = getCurrentPose();

        // Calculate vector from robot to target
        Vector2d relativeTargetVector = new Vector2d(
                targetPoint.x - botPose.getX(),
                targetPoint.y - botPose.getY()

        );

        // Use PID to calculate drive speed
        double driveSpeed = driveController.calculate(0, relativeTargetVector.magnitude());

        // Create movement vector and rotate to robot frame
        Vector2d movementSpeed = new Vector2d(
                driveSpeed,
                relativeTargetVector.angle(),
                true
        ).rotate(-botPose.getHeading().getAngleRadians());

        // Determine target heading
        double targetAngle;
        boolean canFlip = false;

        if (useEndpointHeading && targetPoint.targetEndRotation != null &&
                botPose.getPosition().distanceTo(targetPoint)
                        < 10) {
            targetAngle = targetPoint.targetEndRotation.getAngleRadians();
        } else if (targetPoint.targetFollowRotation != null) {
            targetAngle = targetPoint.targetFollowRotation.getAngleRadians();
        } else if (relativeTargetVector.magnitude() > 5) {
            targetAngle = relativeTargetVector.angle() - Math.PI / 2;
            canFlip = true;
        } else {
            targetAngle = lastTargetAngle;
            canFlip = true;
        }
        lastTargetAngle = targetAngle;

        // Calculate rotation error
        double rotError = Rotation2d.getError(targetAngle, botPose.getHeading().getAngleRadians());
        if (rotError > Math.PI && canFlip) {
            rotError = Rotation2d.getError(targetAngle + Math.PI, botPose.getHeading().getAngleRadians());
        }

        // Limit speed
        double magnitude = Range.clip(movementSpeed.magnitude(),
                -targetPoint.maxVelocity,
                targetPoint.maxVelocity);
        movementSpeed = new Vector2d(magnitude, movementSpeed.angle(), false);

        // Calculate rotation speed
        double rotSpeed = rotController.calculate(0, rotError);

        // Drive!
        drive(movementSpeed.y, movementSpeed.x, rotSpeed);
    }

    /**
     * Find intersection of lookahead circle with path segment (PURE PURSUIT CORE!)
     */
    private static Waypoint intersection(Pose2d botPose, Waypoint[] lineSegment, double radius) {
        double x1, y1, x2, y2;

        double m = (lineSegment[0].y - lineSegment[1].y) / (lineSegment[0].x - lineSegment[1].x);
        double b = lineSegment[0].y - m * lineSegment[0].x;

        double h = botPose.getX();
        double k = botPose.getY();

        double commonTerm;

        if (Double.isFinite(m)) {
            commonTerm = Math.sqrt(
                    Math.pow(m, 2) * (Math.pow(radius, 2) - Math.pow(h, 2)) +
                            (2 * m * h) * (k - b) +
                            2 * b * k +
                            Math.pow(radius, 2) -
                            Math.pow(b, 2) -
                            Math.pow(k, 2)
            );

            x1 = (m * (k - b) + h + commonTerm) / (Math.pow(m, 2) + 1);
            x2 = (m * (k - b) + h - commonTerm) / (Math.pow(m, 2) + 1);

            y1 = m * x1 + b;
            y2 = m * x2 + b;
        } else {
            x1 = lineSegment[0].x;
            commonTerm = Math.sqrt(Math.pow(radius, 2) - Math.pow((x1 - h), 2));
            y1 = botPose.getY() + commonTerm;
            x2 = x1;
            y2 = botPose.getY() - commonTerm;
        }

        Waypoint point0 = new Waypoint(x1, y1, 0,
                lineSegment[1].targetFollowRotation,
                lineSegment[1].targetEndRotation,
                lineSegment[1].maxVelocity);
        Waypoint point1 = new Waypoint(x2, y2, 0,
                lineSegment[1].targetFollowRotation,
                lineSegment[1].targetEndRotation,
                lineSegment[1].maxVelocity);

        Pair<Waypoint, Double> intersection0 = new Pair<>(
                point0, getTValue(lineSegment[0], lineSegment[1], point0));
        Pair<Waypoint, Double> intersection1 = new Pair<>(
                point1, getTValue(lineSegment[0], lineSegment[1], point1));

        Pair<Waypoint, Double> bestIntersection =
                intersection0.second > intersection1.second ? intersection0 : intersection1;

        if (bestIntersection.second > 1) {
            return null;
        }

        return bestIntersection.first;
    }

    private static double getTValue(Vector2d point1, Vector2d point2, Vector2d interpolationPoint) {
        if (Math.abs(point1.x - point2.x) < 0.001) {
            return (interpolationPoint.y - point1.y) / (point2.y - point1.y);
        }
        return (interpolationPoint.x - point1.x) / (point2.x - point1.x);
    }

    /**
     * Main path following method - call this in your loop!
     */
    public void followPath() {
        Pose2d botPose = getCurrentPose();
        Waypoint targetPoint;
        boolean endOfPath = false;

        switch (driveState) {
            case IDLE:
                if (waypointIndex != 0) return;
                followStartTimestamp = timer.milliseconds();
                driveState = DriveState.FOLLOWING;

            case FOLLOWING:
                if (timer.milliseconds() > followStartTimestamp + followPath.timeout) {
                    driveState = DriveState.IDLE;
                    return;
                }

                targetPoint = intersection(botPose, segments[waypointIndex],
                        segments[waypointIndex][1].followRadius);

                if (targetPoint == null) {
                    if (waypointIndex == segments.length - 1) {
                        targetPoint = segments[segments.length - 1][1];
                        endOfPath = true;
                    } else {
                        waypointIndex++;
                        followPath();
                        return;
                    }
                }
                driveToWaypoint(targetPoint, endOfPath);
                break;
        }
    }

    /**
     * Check if path following is complete
     */
    public boolean finishedFollowing() {
        if (timer.milliseconds() > followStartTimestamp + followPath.timeout &&
                timer.milliseconds() > followStartTimestamp + 1) {
            return true;
        }

        Pose2d botPose = getCurrentPose();
        double currentTimestamp = timer.milliseconds();

        double speed = botPose.getPosition().distanceTo(lastPose.getPosition())
                /
                (currentTimestamp - lastTimestamp) * 1000;

        lastTimestamp = currentTimestamp;

        boolean atEndpoint = speed < 2.0 &&
                botPose.getPosition().distanceTo(segments[segments.length - 1][1]) < 2.0 &&
                waypointIndex == segments.length - 1;

        boolean atTargetHeading;
        if (segments[segments.length - 1][1].targetEndRotation == null) {
            atTargetHeading = true;
        } else {
            atTargetHeading = Math.abs(Rotation2d.getError(
                    segments[segments.length - 1][1].targetEndRotation.getAngleRadians(),
                    botPose.getHeading().getAngleRadians()
            )) < Math.toRadians(5);
        }

        lastPose = botPose;
        return atEndpoint && atTargetHeading;
    }

    /**
     * Get remaining distance to end of path
     */
    public double remainingDistance() {
        if (driveState == DriveState.IDLE) {
            return 0;
        }
        Pose2d currentPose = getCurrentPose();
        double distance = currentPose.getPosition().distanceTo(segments[waypointIndex][1]);
        for (int i = waypointIndex + 1; i < segments.length; i++) {
            distance += segments[i][0].distanceTo(segments[i][1]);
        }
        return distance;
    }

    private static class Pair<T, U> {
        public final T first;
        public final U second;

        public Pair(T first, U second) {
            this.first = first;
            this.second = second;
        }
    }

    public void update()
    {
        poseEstimator.update();
    }
    public void updateValues(Telemetry telemetry)
    {
        telemetry.addData("Robot backOdo", backOdo.getAsInt() * poseEstimator.centimetersPerTick);
        telemetry.addData("Robot rightOdo", rightOdo.getAsInt() * poseEstimator.centimetersPerTick);
        telemetry.addData("Robot leftOdo", leftOdo.getAsInt() * poseEstimator.centimetersPerTick);
        telemetry.addData("Robot GlobalX", poseEstimator.getGlobalX());
        telemetry.addData("Robot GlobalY", poseEstimator.getGlobalY());
        telemetry.addData("Robot backOdo ticks ", backOdo.getAsInt());
        telemetry.addData("Robot rightOdo ticks", rightOdo.getAsInt());
        telemetry.addData("Robot leftOdo ticks", leftOdo.getAsInt());
    }
}