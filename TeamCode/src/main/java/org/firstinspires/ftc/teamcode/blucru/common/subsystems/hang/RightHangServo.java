package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

public class RightHangServo extends HangServo{
    public RightHangServo() {
        super("right hang");
    }

    @Override
    public void retract() {
        setPosition(0.87);
    }

    @Override
    public void release() {
        setPosition(0);
    }

    @Override
    public void midway() {
        setPosition(0.5);
    }

    @Override
    public void hang() {
        setPosition(0.91);
    }
}
