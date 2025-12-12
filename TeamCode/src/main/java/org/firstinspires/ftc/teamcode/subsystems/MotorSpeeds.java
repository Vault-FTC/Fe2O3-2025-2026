package org.firstinspires.ftc.teamcode.subsystems;

public enum MotorSpeeds {
    ZERO(0),

    NEAR(850),
    FAR(0),
//    THREE_EIGHTS(1000),
AUTO_NEAR(400),
    ONE_TWENTIETH(200),

    ONE_FIFTEENTHS(300),
    ONE_TENTH(400),
     ONE_EIGHTH(600),
    ONE_FIFTH(800),

//    FIVE_EIGHTS(1200),
//    THREE_QUARTERS(1500),
    THREE_FOURTHS(1200),
//    NINETY(5400),
//    NINETY_FIVE(1400),
    FULL(2000);

    public final double speed;

    MotorSpeeds(double val)
    {
        speed = val;
    }
}
