package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class ServoGate extends Subsystem {
    private Servo dweeb;
   //
    public ServoGate(HardwareMap hardwareMap) {
        dweeb = hardwareMap.get(Servo.class, "servos");
   //     kicker = hardwareMap.get(DcMotor.class, "kicker");
    }
    public void closeGate() {
        dweeb.setPosition(0);
    }
 //   public void spinKicker(double power) {
   //     kicker.setPower(power);
    public void OpenGate()  {
   dweeb.setPosition(1);    
    }
}