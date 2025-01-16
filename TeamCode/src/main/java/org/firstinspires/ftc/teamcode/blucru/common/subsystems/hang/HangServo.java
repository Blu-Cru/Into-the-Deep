package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public abstract class HangServo extends BluServo {
    public HangServo(String name) {
        super(name);
    }

    public abstract void retract();
    public abstract void release();
    public abstract void hang();
}
