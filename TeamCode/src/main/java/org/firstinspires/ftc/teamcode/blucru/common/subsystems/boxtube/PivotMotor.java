package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class PivotMotor extends BluMotorWithEncoder {
    static final double TICKS_PER_RAD = 680.435;

    public PivotMotor() {
        super("pivot", Direction.REVERSE);
    }

    public double getAngle() {
        return getCurrentPosition() / TICKS_PER_RAD;
    }

    public double getAngleVel() {
        return getVelocity() / TICKS_PER_RAD;
    }

    public Vector2d getState() {
        return new Vector2d(getAngle(), getAngleVel());
    }

    @Override
    public void telemetry() {
        Globals.tele.addData("Pivot power", super.getPower());
        Globals.tele.addData("Pivot Angle", getAngle());
        Globals.tele.addData("Pivot Ang. Vel", getAngleVel());
    }
}
