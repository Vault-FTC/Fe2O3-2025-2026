package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double kP, kI, kD;

    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        lastTime = timer.seconds();
    }

    /**
     * @param target desired value
     * @param current measured value
     * @return PID output
     */
    public double calculate(double target, double current) {
        double error = target - current;

        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt <= 0) return 0;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
