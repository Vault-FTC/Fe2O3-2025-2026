package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.MotorSpeeds;
import org.firstinspires.ftc.teamcode.Autonomous.Location;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.ServoGate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.driveallclass;


// ALL SHOOTER SPEEDS ARE IN TICKS/SECOND. DO NOT, I REPEAT DO NOT, USE DEGREES/SECOND
@TeleOp(name = "TeleOp Blue", group = "Teleop")
public class SimpleFieldCentricDrive extends LinearOpMode {

    public LimeLight Limelight;
    Intake intake;
    boolean last_triangle;
    boolean last_up;
    boolean last_down;
    boolean shooting;
    double feedPulseInterval = 0.05; //seconds for feed/pause
    double lastFeedToggleTime = 0;
    boolean feeding = false;
    RevBlinkinLedDriver.BlinkinPattern green;
    RevBlinkinLedDriver.BlinkinPattern red;

    Lights light;

    double launchpower = (0);

    public void setTargets() {
        Limelight = new LimeLight(hardwareMap, 20);
    }

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        light = new Lights(hardwareMap);
        driveallclass drive = new driveallclass(hardwareMap);
        ServoGate servoGate = new ServoGate(hardwareMap);
        Shooter launcher = new Shooter(hardwareMap);
        launchpower = 1200;
        setTargets();
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        red = RevBlinkinLedDriver.BlinkinPattern.RED;

        waitForStart();
        // poseEstimator.update();
        while (opModeIsActive()) {
            double currentTime = getRuntime();

            double joystick_y = gamepad1.left_stick_y; // Forward/backward
            double joystick_x = gamepad1.left_stick_x;  // Strafe left/right
            double joystick_rx = -gamepad1.right_stick_x; // Rotation

            if (gamepad1.start) {
                drive.resetHeading(0);
            }

            if (!last_triangle && gamepad1.y) {
                shooting = !shooting;
            }
//            if (!shooting && drive.isAtPosition())
//            {
//                spinning up the flywheel when you're close to launch areas.
//            }
            last_triangle = gamepad1.y;
            if (gamepad1.x) {
//                intake.spinIntake(0.675);
                if (Math.abs(launcher.getShooterVelocity() - launchpower) < 300) {
                    feeding = !feeding;
                    if (feeding) {
                        intake.spinIntake(0.7);
                    } else{
                        intake.spinIntake(0);
                    }
                    lastFeedToggleTime = currentTime;
                }
                servoGate.openGate();
            } else if (gamepad1.left_bumper) {
                intake.spinIntake(0.95);

            } else if (gamepad1.b) {
                intake.spinIntake(-0.95);
                launcher.execute(true, -900);
            } else {
                intake.spinIntake(0);
                launcher.setPower(0);
                servoGate.closeGate();
            }


            if (gamepad1.right_bumper) {
                LLResultTypes.FiducialResult result = Limelight.getResult();
                if (result == null) {

                } else {
                    joystick_rx = joystick_rx - Limelight.getTx() / 2.5;
                    gamepad1.rumble(1000);
                    double range = Math.abs(result.getCameraPoseTargetSpace().getPosition().z);
                    // launchpower = 0.4 + range / 4;
                    // was 0.3
//                    this.launchpower = launcher.distanceToSpeed(range);
                    telemetry.addData("fff", range);
                    if (result.getCameraPoseTargetSpace().getPosition().x < 67) {
                        light.setColor(green);
                        if (result.getCameraPoseTargetSpace().getPosition().z >= -2.5) {
                            this.launchpower = 1400;
                            feedPulseInterval = 0.1;
                        }
                        else {
                            this.launchpower = 1800;
                            feedPulseInterval = 0.2;
                        }
                        launcher.setShooterVelocityDynamic(this.launchpower);
                    } else {
                        light.setColor(red);
                    }
                }
            }
            else {
                launcher.execute(false, 0);
            }

//            else {
//                if (shooting) {
//                    launcher.setShooterSpeed(-500);
//                } else {
//                    launcher.setShooterSpeed(MotorSpeeds.ZERO.speed);
//                }
//
//                if (gamepad1.x) {
//                    intake.spinIntake(0.8);
//                } else {
//                    intake.spinIntake(0);
//                }
//            }
// A bunch of comments are underneath this but I was tired of seeing them.
//            if (!last_dpad_up && gamepad1.dpad_up) {
//                int newVal = launchpower.ordinal() + 1;
//                if (newVal >= MotorSpeeds.values().length)
//                {
//                    newVal = MotorSpeeds.values().length - 1;
//                }
//                launchpower = MotorSpeeds.values()[newVal];
//            }
//            if (!last_dpad_down && gamepad1.dpad_down) {
//                int newVal = launchpower.ordinal() - 1;
//                if (newVal < 0)
//                {
//                    newVal = 0;
//                }
//                launchpower = MotorSpeeds.values()[newVal];
//            }
//
//            last_dpad_up = gamepad1.dpad_up;
//            last_dpad_down = gamepad1.dpad_down;

            //long shot
//            if (gamepad1.dpad_down && !last_down) {
//                this.launchpower = this.launchpower - 50;
//               //launchpower = MotorSpeeds.NEAR;
//            } //short shot
//            if (gamepad1.dpad_up && !last_up) {
//                this.launchpower = this.launchpower + 50;
//                //launchpower = MotorSpeeds.FAR;
//            }
//            if (gamepad1.dpad_left) {
//                launchpower = MotorSpeeds.FULL;
//            }

            if (gamepad1.back) {
                drive.driveToPosition(new Location(0,0,0), 0 ,telemetry);
            }
            else {
                drive.drive(joystick_y, joystick_x, joystick_rx);
            }


            last_down = gamepad1.dpad_down;
            last_up = gamepad1.dpad_up;
            telemetry.addData("shootSpeed", launcher.getShooterVelocity());
            telemetry.addData("LaunchPower", this.launchpower);
            telemetry.addData("Position", drive.getPosition());
            if (Limelight.getResult() != null) {
                telemetry.addData("Distance from AprilTag", Limelight.getResult().getCameraPoseTargetSpace().getPosition().z);
            }
            telemetry.update();
            drive.updateValues(telemetry);
        }
    }
}