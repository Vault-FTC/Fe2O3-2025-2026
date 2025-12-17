package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MotorSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootCommand extends Command {
    Telemetry telemetry;
    Shooter shooter;
    Intake intake;
    MotorSpeeds motorSpeed;
    boolean shoot;
    private double startTime;

    public ShootCommand(Shooter shooter, Intake intake, boolean shoot, Telemetry telemetry, MotorSpeeds motorSpeed) {
        this.shooter = shooter;
        this.intake = intake;
        this.telemetry = telemetry;
        this.motorSpeed = motorSpeed;
        this.shoot = shoot;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeed(motorSpeed.speed);
        timer.reset();
        startTime = timer.milliseconds();
    }
    @Override
    public void execute() {
        double elapsed = timer.milliseconds() - startTime;
        if (elapsed > 2000) {
            intake.spinIntake(0.85);
        } else {
            intake.spinIntake(0);
            shooter.execute(true, motorSpeed.speed);
        }
        telemetry.addData("Running", "Shoot Command");
    }

    public boolean isFinished() {
        return shoot;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeed(MotorSpeeds.ZERO.speed);
        intake.spinIntake(0);
  //      intake.spinKicker(0);
    }
}
