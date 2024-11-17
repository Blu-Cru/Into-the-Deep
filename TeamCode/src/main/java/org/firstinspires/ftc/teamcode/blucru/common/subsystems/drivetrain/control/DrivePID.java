package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Config
public class DrivePID {
    public static double
            kPX = 0.18, kIX = 0, kDX = 0.021,
            kPY = 0.18, kIY = 0, kDY = 0.021,
            kPHeading = 1.7, kIHeading = 0, kDHeading = 0.1;

    PIDController xController, yController, headingController;

    public DrivePID() {
        xController = new PIDController(kPX, kIX, kDX);
        yController = new PIDController(kPY, kIY, kDY);
        headingController = new PIDController(kPHeading, kIHeading, kDHeading);
    }

    public Vector2d calculate(Vector2d currentPosition) {
        double xPower = xController.calculate(currentPosition.getX());
        double yPower = yController.calculate(currentPosition.getY());
        return new Vector2d(xPower, yPower);
    }

    public Pose2d calculate(Pose2d currentPose) {
        double xPower = Range.clip(xController.calculate(currentPose.getX()), -1, 1);
        double yPower = Range.clip(yController.calculate(currentPose.getY()), -1, 1);
        double headingPower = getRotate(currentPose.getHeading());
        return new Pose2d(xPower, yPower, headingPower);
    }

    public void setTargetPose(Vector2d targetPosition) {
        xController.setSetPoint(targetPosition.getX());
        yController.setSetPoint(targetPosition.getY());
    }

    public void setTargetPose(Pose2d targetPose) {
        xController.setSetPoint(targetPose.getX());
        yController.setSetPoint(targetPose.getY());
        headingController.setSetPoint(targetPose.getHeading());
    }

    public void updatePID() {
        xController.setPID(kPX, kIX, kDX);
        yController.setPID(kPY, kIY, kDY);
        headingController.setPID(kPHeading, kIHeading, kDHeading);
    }

    public void setTargetHeading(double targetHeading) {
        headingController.setSetPoint(targetHeading);
    }

    public double getRotate(double heading) {
        if(heading - headingController.getSetPoint() < -Math.PI) heading += 2*Math.PI;
        else if(heading - headingController.getSetPoint() > Math.PI) heading -= 2 * Math.PI;

        return Range.clip(headingController.calculate(heading), -1, 1);
    }

    public boolean inRange(Pose2d pose, double translationTolerance, double headingTolerance) {
        Vector2d targetVec = new Vector2d(xController.getSetPoint(), yController.getSetPoint());
        boolean translationInRange = pose.vec().minus(targetVec).norm() < translationTolerance;

        double angleDelta = Angle.normDelta(pose.getHeading() - headingController.getSetPoint());
        boolean headingInRange = angleDelta < headingTolerance;

        return translationInRange && headingInRange;
    }
}
