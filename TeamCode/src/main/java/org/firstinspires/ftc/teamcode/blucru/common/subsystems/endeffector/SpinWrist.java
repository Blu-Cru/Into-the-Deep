package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public class SpinWrist extends BluServo {
    public static double
            // positive turns clockwise when intaking
            CENTER_POS = 0.38;
    static final double
            // 1 tick = range/360 of a full rotation
            TICKS_PER_RAD = 0.28/(Math.PI/2),
            MAX_ANGLE = Math.PI/2, MIN_ANGLE = -Math.PI/2;

    public SpinWrist() {
        super("spinwrist");
    }

    public void setAngle(double rad) {
        double angle = Range.clip(rad, MIN_ANGLE, MAX_ANGLE);
        super.setPosition(toTicks(angle));
    }

    double toTicks(double rad) {
        return rad * TICKS_PER_RAD; // Convert radians to a value between 0 and 1
    }
}
