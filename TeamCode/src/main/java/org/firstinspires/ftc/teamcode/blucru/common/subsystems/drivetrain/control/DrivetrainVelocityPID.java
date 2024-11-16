package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;

public class DrivetrainVelocityPID {
    double kP;
    double kI;
    double kD;
    double tolerance;

    PIDController xController = new PIDController(kP, kI, kD);
    PIDController yController = new PIDController(kP, kI, kD);

    Vector2d targetVelocity;

    public DrivetrainVelocityPID(double kP, double kI, double kD, double tolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;

        xController = new PIDController(kP, kI, kD);
        yController = new PIDController(kP, kI, kD);

        targetVelocity = new Vector2d(0, 0);
    }

    public Vector2d calculateDelta(Vector2d currentVelocity, double maxDeltaMagnitude) {
        double xDeltaPower = xController.calculate(currentVelocity.getX(), targetVelocity.getX());
        double yDeltaPower = yController.calculate(currentVelocity.getY(), targetVelocity.getY());

        Vector2d rawDeltaDriveVector = new Vector2d(xDeltaPower, yDeltaPower);
        double powerMagnitude = rawDeltaDriveVector.norm();
        double limitedMagnitude = Math.min(powerMagnitude, maxDeltaMagnitude);
        Vector2d deltaDriveVector = rawDeltaDriveVector.div(powerMagnitude).times(limitedMagnitude);

        return deltaDriveVector;
    }

    public void setTargetVelocity(Vector2d targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        xController.setP(kP);
        xController.setI(kI);
        xController.setD(kD);
        yController.setP(kP);
        yController.setI(kI);
        yController.setD(kD);
    }

    public void setkP(double kP) {
        this.kP = kP;
        xController.setP(kP);
        yController.setP(kP);
    }

    public void setkI(double kI) {
        this.kI = kI;
        xController.setI(kI);
        yController.setI(kI);
    }

    public void setkD(double kD) {
        this.kD = kD;
        xController.setD(kD);
        yController.setD(kD);
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
}
