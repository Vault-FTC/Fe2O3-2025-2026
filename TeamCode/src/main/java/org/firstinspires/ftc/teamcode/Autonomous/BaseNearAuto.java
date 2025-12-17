package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandSystem.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandSystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LimeLightTurnCommand;
import org.firstinspires.ftc.teamcode.Commands.TimedShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.MotorSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.driveallclass;

@Autonomous (name = "Blue Near", group = "Blue Team")
public class BaseNearAuto extends LinearOpMode {
    driveallclass drive;
    Shooter shooter;
    Intake intake;
    LimeLight LimeLight;
    ServoGate servoGate;
    Location launchPosition = new Location(-110, 10, 0);
    Location collectFirstRowArtifacts = new Location(-70, -60, 43);
    Location hitGate = new Location(-65, -105, -48);
    Location prepareSecondRowArtifacts = new Location(-145,-80, 43);
    Location collectSecondRowArtifacts = new Location(-88, -135, 43);
    Location prepareCollectThirdRowArtifacts = new Location(-184,-124, 43);
    Location collectionThirdRowArtifacts = new Location(-124,-175, 43);
    Location lastLaunchPosition = new Location(-110, 20, 0);
    Location leaveZonePosition = new Location(-80, -70, 43);


    CommandScheduler scheduler = CommandScheduler.getInstance();
    Command auto;

    void setTargets() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new driveallclass(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        LimeLight = new LimeLight(hardwareMap,20);
        servoGate = new ServoGate(hardwareMap);
        scheduler.clearRegistry();

        setTargets();

        SequentialCommandGroup auto = SequentialCommandGroup.getBuilder()
                .add(new DriveToCommand(drive, launchPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive, LimeLight, telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1350, servoGate))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectFirstRowArtifacts, telemetry))
                        .build()
                )
                .add(new DriveToCommand(drive, hitGate, telemetry))
                .add(new DriveToCommand(drive, launchPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight,telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1350, servoGate))
                .add(new DriveToCommand(drive, prepareSecondRowArtifacts, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectSecondRowArtifacts, telemetry))
                        .build()
                )
                .add(new DriveToCommand(drive, prepareSecondRowArtifacts, telemetry))
                .add(new DriveToCommand(drive, hitGate, telemetry))
                .add(new DriveToCommand(drive, launchPosition, telemetry))
//                .add(new LimeLightTurnCommand(drive,LimeLight,telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1350, servoGate))

                .add(new DriveToCommand(drive, prepareCollectThirdRowArtifacts, telemetry))
                .add(ParallelCommandGroup.getBuilder()
                        .add(new IntakeCommand(intake, 2, telemetry, servoGate))
                        .add(new DriveToCommand(drive, collectionThirdRowArtifacts, telemetry))
                        .build()
                )

                .add(new DriveToCommand(drive, lastLaunchPosition, telemetry))
                .add(new TimedShootCommand(shooter, intake, 3, telemetry, 1350, servoGate))

                .add(new DriveToCommand(drive, leaveZonePosition, telemetry))
                .build();

        waitForStart();

        auto.schedule();
        while(opModeIsActive()) {
            scheduler.run();
            telemetry.addData("Position", drive.getPosition());
            telemetry.update();
        }
    }
}
