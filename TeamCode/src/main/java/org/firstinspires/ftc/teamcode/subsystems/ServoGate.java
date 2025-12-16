package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandSystem.Subsystem;

public class ServoGate extends Subsystem {
    private Servo servo;
   //
    public ServoGate(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo");
    }
    public void closeGate() {
        servo.setPosition(0.5);
    }
    public void openGate()  {
        servo.setPosition(1);
    }
}