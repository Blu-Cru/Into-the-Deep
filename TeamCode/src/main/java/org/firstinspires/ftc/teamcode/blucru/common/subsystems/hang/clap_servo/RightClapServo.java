package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.clap_servo;

public class RightClapServo extends ClapServo {
    public RightClapServo() {
        super("clapright");
    }

    @Override
    double getCenterPosition() {
        return 0.76;
    }

    @Override
    public void retract() {
        setPosition(getCenterPosition() + RETRACT_DELTA);
    }
}
