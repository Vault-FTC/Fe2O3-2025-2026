package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MotorSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class TimedShootCommand extends Command {
    Telemetry telemetry;
    Shooter shooter;
    Intake intake;
    private final ServoGate servoGate;
    double motorSpeed;
    private final double durationMs;
    private double startTime;

    public TimedShootCommand(Shooter shooter, Intake intake, double durationSeconds, Telemetry telemetry, double motorSpeed, ServoGate servoGate) {
        this.shooter = shooter;
        this.intake = intake;
        this.servoGate = servoGate;
        this.telemetry = telemetry;
        this.motorSpeed = motorSpeed;
        this.durationMs = durationSeconds * 1000;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeed(motorSpeed);
        timer.reset();
        startTime = timer.milliseconds();
    }
    @Override
    public void execute() {
        double elapsed = timer.milliseconds() - startTime;
        servoGate.openGate();
        if (elapsed > 2000) {
            intake.spinIntake(0.95);
   //         intake.spinKicker(0.75);
        } else {
            intake.spinIntake(0);
    //        intake.spinKicker(0);
            shooter.execute(true, motorSpeed);
        }
        telemetry.addData("Running", "Shoot Command");
    }

    public boolean isFinished() {
        return timer.milliseconds() - startTime >= durationMs;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeed(MotorSpeeds.ZERO.speed);
        intake.spinIntake(0);
    //    intake.spinKicker(0);
    }

}
