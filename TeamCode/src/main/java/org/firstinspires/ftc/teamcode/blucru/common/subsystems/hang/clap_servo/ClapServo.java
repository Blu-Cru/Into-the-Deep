package org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.clap_servo;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public abstract class ClapServo extends BluServo {
    public static double RETRACT_DELTA = 0.12;

    public ClapServo(String name) {super(name);}

    abstract double getCenterPosition();
    public abstract void retract();
    public void center() {
        setPosition(getCenterPosition());
    };
}
