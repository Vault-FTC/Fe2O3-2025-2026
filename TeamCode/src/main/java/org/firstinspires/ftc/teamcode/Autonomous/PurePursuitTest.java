package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PurePursuit.Path;
import org.firstinspires.ftc.teamcode.PurePursuit.Waypoint;
import org.firstinspires.ftc.teamcode.geometry.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.driveallclass;

@TeleOp
public class PurePursuitTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        driveallclass drive = new driveallclass(hardwareMap);

        Waypoint[] pathPoints = new Waypoint[] {
                new Waypoint(0, 0, 0),
                new Waypoint(50, 0, 0),
                new Waypoint(100, 50,0)
        };

        Path path = new Path(pathPoints);

        drive.setFollowPath(path);

        waitForStart();

        while (opModeIsActive() && !drive.finishedFollowing()) {
            drive.followPath();
            telemetry.addData("Robot Position", drive.getPosition());
        }
        drive.drive(0,0,0);
        telemetry.update();
    }
}
