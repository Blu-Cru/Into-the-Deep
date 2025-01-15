package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

public class LeftHangServo extends HangServo{
    public LeftHangServo() {
        super("left hang");
    }

    @Override
    public void retract() {
        setPosition(0.5);
    }

    @Override
    public void release() {
        setPosition(0.5);
    }

    @Override
    public void hang() {
        setPosition(0.5);
    }
}
