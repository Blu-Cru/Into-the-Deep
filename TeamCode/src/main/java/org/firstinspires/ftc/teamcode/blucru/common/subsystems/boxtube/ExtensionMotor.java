package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class ExtensionMotor extends BluMotorWithEncoder {
    // TODO: calculate value
    static final double AXIS_REV_PER_INCH = 1.0 / 2.0 / Math.PI / 0.82677;
    static final double TICKS_PER_INCH = 27.932*9/4; // 145.1 on the motor

    PivotMotor pivot;

    public ExtensionMotor() {
        super("extension", Direction.FORWARD);
        pivot = null;
    }

    public double getDistance() {
        if(pivot == null) {
            return getCurrentPosition() / TICKS_PER_INCH;
        } else {
            return getCurrentPosition() / TICKS_PER_INCH;
        }
    }

    public double getDistanceVel() {
        if(pivot == null) {
            return getVelocity() / TICKS_PER_INCH;
        } else {
            return getVelocity() / TICKS_PER_INCH;
        }
    }

    public Vector2d getState() {
        return new Vector2d(
                getDistance(),
                getDistanceVel()
        );
    }

    public void setPivot(PivotMotor pivot) {
        this.pivot = pivot;
    }

    @Override
    public void telemetry() {
        Globals.tele.addData("Extension (in)", getDistance());
        Globals.tele.addData("Extension Velocity (in/s)", getDistanceVel());
    }
}