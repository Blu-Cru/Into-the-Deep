package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

@Config
public class DriveKinematics {
    public static double
        AXIAL_DECEL = 0.5, LATERAL_DECEL = 0.5, HEADING_DECEL = 8;

    public static double getHeadingTowardsPoint(Pose2d currentPose, Pose2d targetPose) {
        return Math.atan2(targetPose.getY() - currentPose.getY(),
                            targetPose.getX() - currentPose.getX());
    }

    public static Vector2d getHeadingStateTowardsPoint(Pose2d currentPose, Pose2d targetPose, Pose2d currentVelocity) {
        double heading = getHeadingTowardsPoint(currentPose, targetPose);

        double xDelta = targetPose.getX() - currentPose.getX();
        double yDelta = targetPose.getY() - currentPose.getY();

        double xVel = currentVelocity.getX();
        double yVel = currentVelocity.getY();

        // derive atan(yDelta / xDelta) with respect to time
        double headingVel = (xDelta * yVel - yDelta * xVel) / (xDelta * xDelta + yDelta * yDelta);

        return new Vector2d(heading, headingVel);
    }

    public static Pose2d getStopPose(Pose2d currentPose, Pose2d currentVelocity) {
        Pose2d robotVel = new Pose2d(currentVelocity.vec().rotated(-currentPose.getHeading()), currentVelocity.getHeading());

        // absolute value does the same as Math.signum because its squared, so the sign is preserved
        double robotDeltaX = robotVel.getX() * Math.abs(robotVel.getX()) / (2 * AXIAL_DECEL);
        double robotDeltaY = robotVel.getY() * Math.abs(robotVel.getY()) / (2 * LATERAL_DECEL);
        double robotDeltaHeading = robotVel.getHeading() * Math.abs(robotVel.getHeading()) / (2 * HEADING_DECEL);

        Pose2d robotDeltaPose = new Pose2d(robotDeltaX, robotDeltaY, robotDeltaHeading);

        Pose2d globalDeltaPose = new Pose2d(robotDeltaPose.vec().rotated(currentPose.getHeading()), robotDeltaPose.getHeading());

        return new Pose2d(currentPose.vec().plus(globalDeltaPose.vec()), Angle.norm(currentPose.getHeading() + globalDeltaPose.getHeading()));
    }


    // apply slew rate limiter to drive vector
    // i used it before, but too computationally intensive and slows robot too much i think
    // also theres def a better way to do this, doing it before scaling to drive power makes it so much worse
//    private Vector2d slewRateLimit(Vector2d input, Vector2d lastDriveVector) {
//
//        // calculate the change between the last drive vector and the current drive vector
//        Vector2d driveVectorDelta = input.minus(lastDriveVector);
//
//        double limitedDriveVectorDeltaMagnitude;
//        boolean decelerating = input.norm() < lastDriveVector.norm();
//
//        if(decelerating) {
//            // if we are decelerating, limit the delta to the max decel delta
//            limitedDriveVectorDeltaMagnitude = Range.clip(driveVectorDelta.norm(), 0, (MAX_DECEL_DRIVE_DELTA * dt / 1000.0));
//        } else {
//            // otherwise, limit the delta to the max accel delta
//            limitedDriveVectorDeltaMagnitude = Range.clip(driveVectorDelta.norm(), 0, (MAX_ACCEL_DRIVE_DELTA * dt / 1000.0));
//        }
//
//        // scale the drive vector delta to the limited magnitude
//        Vector2d scaledDriveVectorDelta = driveVectorDelta.div(driveVectorDelta.norm()).times(limitedDriveVectorDeltaMagnitude);
//
//        Vector2d driveVector;
//        if(driveVectorDelta.norm() == 0) // catch divide by zero
//            driveVector = lastDriveVector;
//        else
//            // add the scaled change in drive vector to the last drive vector
//            driveVector = lastDriveVector.plus(scaledDriveVectorDelta);
//
//        // record the drive vector for the next loop
//        lastDriveVector = driveVector;
//
//        return driveVector; // return the new drive vector
//    }
}
