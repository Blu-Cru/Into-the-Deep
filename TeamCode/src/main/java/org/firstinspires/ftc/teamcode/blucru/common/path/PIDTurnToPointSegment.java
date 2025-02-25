package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PIDTurnToPointSegment implements PathSegment {
    Vector2d drivePose, turnToPose;
    double tolerance;
    boolean stopRequired;
    double startTime;

    public PIDTurnToPointSegment(Vector2d drivePose, Vector2d turnToPose, double tolerance, boolean stopRequiredToEnd) {
        this.drivePose = drivePose;
        this.turnToPose = turnToPose;
        this.tolerance = tolerance;
        this.stopRequired = stopRequiredToEnd;
    }

    public PIDTurnToPointSegment(Vector2d drivePose, Vector2d turnToPose, double tolerance) {
        this(drivePose, turnToPose, tolerance, false);
    }

    public PIDTurnToPointSegment(Vector2d drivePose, Vector2d turnToPose) {
        this(drivePose, turnToPose, 1.5);
    }

    @Override
    public boolean isDone() {
        boolean velSatisfied = !stopRequired ||
                Robot.getInstance().dt.vel.vec().norm() < 4.0;

        return Robot.getInstance().dt.inRangeTurnToPoint(drivePose, turnToPose, tolerance, tolerance * 0.1)
                && velSatisfied;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean failed() {
        return false;
    }

    @Override
    public Pose2d getPose() {
        Pose2d currentPose = Robot.getInstance().dt.pose;
        return new Pose2d(currentPose.vec(), Math.atan2(
                turnToPose.getY() - currentPose.getY(),
                turnToPose.getX() - currentPose.getX()
        ));
    }

    @Override
    public void run() {
        Robot.getInstance().dt.pidTurnToPos(drivePose, turnToPose);
    }
}
