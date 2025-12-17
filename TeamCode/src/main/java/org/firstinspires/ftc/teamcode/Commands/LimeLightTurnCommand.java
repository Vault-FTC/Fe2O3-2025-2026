package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandSystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.driveallclass;

public class LimeLightTurnCommand extends Command
{
    driveallclass moved;
    LimeLight light;
    Telemetry tele;
    ElapsedTime tenseistime = new ElapsedTime();

    public LimeLightTurnCommand(driveallclass move, LimeLight lime, Telemetry tele){
        moved = move;
        light = lime;
        this.tele = tele;
    }

    @Override
    public void execute() {
        LLResultTypes.FiducialResult res = light.getResult();
        if(res == null)
        {
            moved.drive(0,0, 0);
            return;
        }
        Pose3D pose = res.getRobotPoseTargetSpace();
        double p = 0.05;
        double x = (0 + pose.getPosition().x) * p; // correct = 0;
        double z = (-1.8 + pose.getPosition().z) * p;  // correct = -1.8;
        double yaw = light.getTx();
        moved.drive(0,0, yaw / 13);
        tele.addData("yaw", yaw);
    }

    @Override
    public void initialize() {
        tenseistime.reset();
    }

    @Override
    public boolean isFinished(){
       return tenseistime.seconds() > 100;

    }
}
