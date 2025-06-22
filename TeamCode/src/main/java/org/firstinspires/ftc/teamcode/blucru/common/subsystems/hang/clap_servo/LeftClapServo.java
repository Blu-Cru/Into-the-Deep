package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.clap_servo;

public class LeftClapServo extends ClapServo {
    public LeftClapServo() {
        super("clapleft");
    }

    @Override
    double getCenterPosition() {
        return 0.52;
    }

    @Override
    public void retract() {
        setPosition(getCenterPosition() - RETRACT_DELTA);
    }
}
