package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class Shooter extends Subsystem {
 //   boolean spinKicker = true;
    MotorSpeeds currentSpeed = MotorSpeeds.NEAR;
//    private DcMotorEx kicker;
    private DcMotorEx shooter;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20.0004, 0, 0, 17.9967);
    public Shooter(HardwareMap hardwareMap) {
     //   kicker = hardwareMap.get(DcMotorEx.class, "kicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter.setPower(1.0);
        shooter.setVelocity(0, AngleUnit.DEGREES);
    }


//    public void setShooterSpeed(MotorSpeeds speed){
//        currentSpeed = speed;
//        shooter.setVelocity(-currentSpeed.speed, AngleUnit.DEGREES);
//    }


//    public void execute(boolean shoot, MotorSpeeds motorSpeed) {
//        if (shoot) {
//
//        } else {
//            shooter.setVelocity(0);
//      //      kicker.setPower(0);
//        }
//    }

    public double distanceToSpeed(double distanceCm)
    {
        return  ((distanceCm * 100.0 / 2.54) - 40) * 5 + 700;
        //return (585 * Math.pow(1.0046834253, (distanceCm / 2.54)));
    }

    public void execute(boolean shoot, double motorSpeed) {
        if (shoot) {
            shooter.setVelocity(motorSpeed);
        } else {
            shooter.setVelocity(0);
       //     kicker.setPower(0);
        }
    }

    public void setShooterSpeed(double speed){
        //currentSpeed = speed;
        shooter.setVelocity(-speed, AngleUnit.DEGREES);
    }

    public void setShooterSpeedFromAprilTag() {

    }
   // public void toggleKicker(double speed)
    {
  //      kicker.setPower(speed);
    }
//    public void execute(){
////        if (Math.abs(getGateError()) > .1){
////            servo.setPosition(gateClosed ? 1. : 0.);
////        }
//        if (spinShooter){
//            shooter.setVelocity(-shooterSpeed, AngleUnit.DEGREES);
//            //kicker.setPower(-kickSpeed);
//        } else {
//            shooter.setVelocity(0);
//            //kicker.setPower(0);
//        }
//    }

    public double getShooterVelocity()
    {
        return shooter.getVelocity(AngleUnit.DEGREES);
    }

    private double getGateError(){
        return 0;
//        double target = gateClosed ? 1. : 0.;
//        return target - servo.getPosition();
    }
}

