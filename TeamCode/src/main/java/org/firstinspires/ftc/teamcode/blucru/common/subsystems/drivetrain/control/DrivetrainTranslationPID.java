package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;

public class DrivetrainTranslationPID {
    double kP;
    double kI;
    double kD;
    double tolerance;

    PIDController xController, yController;

    Vector2d targetPosition;

    public DrivetrainTranslationPID(double kP, double kI, double kD, double tolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;

        xController = new PIDController(kP, kI, kD);
        yController = new PIDController(kP, kI, kD);

        targetPosition = new Vector2d(0, 0);
    }

    public Vector2d calculate(Vector2d currentPosition) {
        Vector2d error = targetPosition.minus(currentPosition);
        if(error.norm() < tolerance) {
            return new Vector2d(0,0);
        } else {
            double xPower = xController.calculate(currentPosition.getX(), targetPosition.getX());
            double yPower = yController.calculate(currentPosition.getY(), targetPosition.getY());
            return new Vector2d(xPower, yPower);
        }
    }

    public void setTargetPosition(Vector2d targetPosition) {
        this.targetPosition = targetPosition;
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
}
