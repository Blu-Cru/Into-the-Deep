package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

@Config
public abstract class HangServo extends BluServo {
    public static double RETRACT_DELTA = 0.26,
        HANG_DELTA = 0.15,
        RELEASE_DELTA = 0.61,
        MIDWAY_DELTA = 0.26;
    public HangServo(String name) {
        super(name);
    }

    abstract double getSidewaysPosition();
    public abstract void retract();
    public abstract void release();
    public abstract void midway();
    public abstract void hang();
}
