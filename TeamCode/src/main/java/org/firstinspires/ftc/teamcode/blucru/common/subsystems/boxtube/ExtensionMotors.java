package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class ExtensionMotors extends BluMotorWithEncoder {
    // TODO: calculate value
    static final double AXIS_RAD_PER_INCH = 1.0 / 0.82677;
    static final double TICKS_PER_INCH = 27.932; // 145.1 on the motor

    BluMotor extension2;
    PivotMotor pivot;

    public ExtensionMotors() {
        super("extension1", Direction.FORWARD);
        extension2 = new BluMotor("extension2", Direction.FORWARD);
        pivot = null;
    }

    public double getDistance() {
        if(pivot == null) {
            return getCurrentPosition() / TICKS_PER_INCH;
        } else {
            return (getCurrentPosition() / TICKS_PER_INCH) + (pivot.getAngle() / AXIS_RAD_PER_INCH);
        }
    }

    public double getDistanceVel() {
        if(pivot == null) {
            return getVelocity() / TICKS_PER_INCH;
        } else {
            return getVelocity() / TICKS_PER_INCH + pivot.getAngleVel() / AXIS_RAD_PER_INCH;
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

    public void setPower(double power) {
        super.setPower(power);
        extension2.setPower(power);
    }

    public void write() {
        super.write();
        extension2.write();
    }

    @Override
    public void telemetry() {
        Globals.tele.addData("Extension (in)", getDistance());
        Globals.tele.addData("Extension Velocity (in/s)", getDistanceVel());
    }
}