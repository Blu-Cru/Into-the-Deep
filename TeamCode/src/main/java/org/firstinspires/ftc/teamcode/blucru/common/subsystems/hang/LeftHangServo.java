package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

public class LeftHangServo extends HangServo{
    public LeftHangServo() {
        super("left hang");
    }

    @Override
    public void retract() {
        setPosition(0.14);
    }

    @Override
    public void release() {
        setPosition(1);
    }

    @Override
    public void hang() {
        setPosition(0.1);
    }
}
