package org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.SmoothServo;

public class UpDownWrist extends SmoothServo {
    // positive is more down
    public static double PARALLEL_POS = 0.435;
    final double TICKS_PER_RAD = 0.275 / (Math.PI / 2);

    public UpDownWrist() {
        super("updownwrist", false);
    }
}
