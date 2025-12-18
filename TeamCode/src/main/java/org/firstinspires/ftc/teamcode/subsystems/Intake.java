package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class Intake extends Subsystem {
    private DcMotorEx intake;
   //
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake1");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void spinIntake(double power) {
        intake.setPower(power);
    }
 //   public void spinKicker(double power) {
   //     kicker.setPower(power);
    }
