package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Location;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MotorSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.driveallclass;

public class IntakeCommand extends Command {
    Telemetry telemetry;
    private final Intake intake;
    private final double durationMs;
    private double startTime;

    public IntakeCommand(Intake intake, double durationSeconds, Telemetry telemetry) {
        this.intake = intake;
        this.durationMs = durationSeconds * 1000;
        this.telemetry = telemetry;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
        startTime = timer.milliseconds();
    }

    @Override
    public void execute() {
        intake.spinIntake(0.75);
 //       intake.spinKicker(-0.75);
        telemetry.addData("Running", "Intake Command");
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() - startTime >= durationMs;
    }

    @Override
    public void end(boolean interrupted) {
        intake.spinIntake(0);
  //      intake.spinKicker(0);
    }
}
