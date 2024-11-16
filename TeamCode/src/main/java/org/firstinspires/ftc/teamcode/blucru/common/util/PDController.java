package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;

public class PDController extends PIDController {
    Vector2d k;
    public PDController(double kP, double kI, double kD) {
        super(kP, kI, kD);
        k = new Vector2d(kP, kD);
    }

    public double calculate(double currentPos, double targetPos, double currentVelocity, double targetVelocity) {
        if(targetVelocity == 0) {
            return calculate(currentPos, targetVelocity);
        }

        Vector2d pv = new Vector2d(currentPos, currentVelocity);
        Vector2d sp = new Vector2d(targetPos, targetVelocity);

        return calculate(pv, sp);
    }

    public double calculate(Vector2d pv, Vector2d sp) {
        Vector2d error = sp.minus(pv);
        return error.dot(k);
    }

    public double calculate(Vector2d pv, MotionProfile profile) {
        Vector2d sp = profile.getInstantState();
        return calculate(pv, sp);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        super.setPID(kp, ki, kd);
        k = new Vector2d(kp, kd);
    }
}