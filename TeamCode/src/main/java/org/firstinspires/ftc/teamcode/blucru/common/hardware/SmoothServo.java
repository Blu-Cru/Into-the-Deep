package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class SmoothServo extends BluServo implements BluHardwareDevice {
    final static double defaultKPrev = 0.03;

    /*

    A kPrev of 0 will result in no smoothing,
    while a kPrev of 1 will result in most smoothing

    Lower kPrev = less smoothing
    Higher kPrev = more smoothing

    A decent range for kPrev is 0 to 0.05

    keep in mind, a smooth servo will continuously
    be updating position, which results in longer loop times

     */

    double kPrev; // weight of previous position in smoothing
    double targetPosition;

    public SmoothServo(String name, Direction direction) {
        super(name, direction);
        kPrev = defaultKPrev;
    }

    public SmoothServo(String name) {
        super(name);
        kPrev = defaultKPrev;
    }

    public SmoothServo(String name, double kPrev) {
        super(name);
        this.kPrev = kPrev;
    }

    public SmoothServo(String name, Direction direction, double kPrev) {
        super(name, direction);
        this.kPrev = kPrev;
    }

    public void init() {
        super.init();
    }

    public void read() {
        super.read();
    }

    public void write() {
        super.setPosition(kPrev * pos + (1 - kPrev) * targetPosition);
        super.write();
    }

    public void setPosition(double position) {
        targetPosition = position;
    }

    public void telemetry() {
        Globals.tele.addData(name + "target pos", targetPosition);
        super.telemetry();
    }
}