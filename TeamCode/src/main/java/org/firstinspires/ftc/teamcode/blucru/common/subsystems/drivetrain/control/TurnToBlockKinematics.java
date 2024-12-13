package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class TurnToBlockKinematics {
    public Vector2d blockPos;

    public TurnToBlockKinematics(Vector2d blockPos) {
        this.blockPos = blockPos;
    }

    public double getHeadingTowardsPoint(Pose2d currentPose) {
        return Math.atan2(blockPos.getY() - currentPose.getY(),
                blockPos.getX() - currentPose.getX());
    }

    public Vector2d getHeadingStateTowardsPoint(Pose2d currentPose, Pose2d currentVelocity) {
        double heading = getHeadingTowardsPoint(currentPose);

        double xDelta = blockPos.getX() - currentPose.getX();
        double yDelta = blockPos.getY() - currentPose.getY();

        double xVel = currentVelocity.getX();
        double yVel = currentVelocity.getY();

        // derive atan(yDelta / xDelta) with respect to time
        double headingVel = (xDelta * yVel - yDelta * xVel) / (xDelta * xDelta + yDelta * yDelta);

        return new Vector2d(heading, headingVel);
    }

    public Vector2d getExtensionStateTowardsPoint(Pose2d currentPose, Pose2d currentVelocity) {
        double xDelta = blockPos.getX() - currentPose.getX();
        double yDelta = blockPos.getY() - currentPose.getY();

        double xVel = currentVelocity.getX();
        double yVel = currentVelocity.getY();

        double xAccel = currentVelocity.getX();
        double yAccel = currentVelocity.getY();

        double extension = Math.sqrt(xDelta * xDelta + yDelta * yDelta);
        // derive sqrt(xDelta^2 + yDelta^2) with respect to time
        double extensionVel = (xDelta * xVel + yDelta * yVel) / extension;

        return new Vector2d(extension, extensionVel);
    }
}
