package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.slides;

public class LeftSlideHangServo extends SlideHangServo {
    public LeftSlideHangServo() {
        super("left hang");
    }

    @Override
    double getSidewaysPosition() {
        return 0.39;
    }

    @Override
    public void retract() {
        setPosition(getSidewaysPosition() - RETRACT_DELTA);
    }

    @Override
    public void release() {
        setPosition(getSidewaysPosition() + RELEASE_DELTA);
    }

    @Override
    public void midway() {
        setPosition(getSidewaysPosition() + MIDWAY_DELTA);
    }

    @Override
    public void hang() {
        setPosition(getSidewaysPosition() - RETRACT_DELTA);
    }
}
