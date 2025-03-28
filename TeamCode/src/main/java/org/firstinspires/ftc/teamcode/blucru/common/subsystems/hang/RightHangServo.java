package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang;

public class RightHangServo extends HangServo{
    public RightHangServo() {
        super("right hang");
    }

    @Override
    double getSidewaysPosition() {
        return 0.58;
    }

    @Override
    public void retract() {
        setPosition(getSidewaysPosition() + RETRACT_DELTA);
    }

    @Override
    public void release() {
        setPosition(getSidewaysPosition() - RELEASE_DELTA);
    }

    @Override
    public void midway() {
        setPosition(getSidewaysPosition() - MIDWAY_DELTA);
    }

    @Override
    public void hang() {
        setPosition(getSidewaysPosition() + HANG_DELTA);
    }
}
