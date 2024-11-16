package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class ExtensionMotor extends BluMotorWithEncoder {
    // TODO: calculate value
    static final double TICKS_PER_INCH = 27.932*9/4; // 145.1 on the motor

    public ExtensionMotor() {
        super("extension", Direction.FORWARD);
    }

    public double getDistance() {
        return getCurrentPosition() / TICKS_PER_INCH;
    }

    public double getDistanceVel() {
        return getVelocity() / TICKS_PER_INCH;
    }

    @Override
    public void telemetry() {
        Globals.tele.addData("Extension (in)", getDistance());
        Globals.tele.addData("Extension Velocity (in/s)", getDistanceVel());
    }
}