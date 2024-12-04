package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.Range;

@Config
public class DriveKinematics {
    public static double
        AXIAL_DECEL = 0.5, LATERAL_DECEL = 0.5, HEADING_DECEL = 8,
        LATERAL_MULTIPLIER = 1.8,
        STRAFE_kStatic = 0.02, FORWARD_kStatic = 0.01, // feedforward constants for static friction
        MAX_ACCEL_DRIVE_DELTA = 0.5, MAX_DECEL_DRIVE_DELTA = 0.5;

    public static double getDistanceToPoint(Pose2d pose, Pose2d targetPose) {
        return Math.hypot(targetPose.getX() - pose.getX(), targetPose.getY() - pose.getY());
    }

//    public static double getDistanceVelToPoint(Pose2d pose, Pose2d targetPose, Pose2d velocity) {
//
//    }

    public static Pose2d getStopPose(Pose2d pose, Pose2d fieldVel) {
        Pose2d robotVel = new Pose2d(fieldVel.vec().rotated(-pose.getHeading()), fieldVel.getHeading());

        // absolute value does the same as Math.signum because it preserves the sign
        double robotDeltaX = robotVel.getX() * Math.abs(robotVel.getX()) / (2 * AXIAL_DECEL);
        double robotDeltaY = robotVel.getY() * Math.abs(robotVel.getY()) / (2 * LATERAL_DECEL);

        Pose2d robotDeltaPose = new Pose2d(robotDeltaX, robotDeltaY, getHeadingDecel(pose.getHeading(), fieldVel.getHeading()));

        Pose2d globalDeltaPose = new Pose2d(robotDeltaPose.vec().rotated(pose.getHeading()), robotDeltaPose.getHeading());

        return new Pose2d(pose.vec().plus(globalDeltaPose.vec()), Angle.norm(pose.getHeading() + globalDeltaPose.getHeading()));
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

    public static Pose2d processStaticFriction(Pose2d drivePose) {
        Vector2d driveVector = drivePose.vec();

        if(driveVector.norm() != 0) {
            double angle = driveVector.angle();
            double staticMinMagnitude =
                    STRAFE_kStatic * FORWARD_kStatic
                            /
                            Math.hypot(STRAFE_kStatic * Math.cos(angle), FORWARD_kStatic * Math.sin(angle));
            double newDriveMagnitude = staticMinMagnitude + (1-staticMinMagnitude) * driveVector.norm();
            return new Pose2d(driveVector.div(driveVector.norm()).times(newDriveMagnitude), drivePose.getHeading());
        } else return drivePose;
    }

    public static double getHeadingDecel(double heading, double headingVel) {
        double headingDelta = headingVel * Math.abs(headingVel) / (2 * HEADING_DECEL);
        return Angle.norm(heading + headingDelta);
    }

    public static double[] getDrivePowers(Pose2d drivePose) {
        // clip to power 1
        drivePose = new Pose2d(Range.clip(drivePose.getX(), -1, 1),
                Range.clip(drivePose.getY(), -1, 1),
                Range.clip(drivePose.getHeading(), -1, 1));

        double[] powers = new double[4];

        powers[0] = drivePose.getX() - LATERAL_MULTIPLIER * drivePose.getY() - drivePose.getHeading();
        powers[1] = drivePose.getX() + LATERAL_MULTIPLIER * drivePose.getY() + drivePose.getHeading();
        powers[2] = drivePose.getX() + LATERAL_MULTIPLIER * drivePose.getY() - drivePose.getHeading();
        powers[3] = drivePose.getX() - LATERAL_MULTIPLIER * drivePose.getY() + drivePose.getHeading();

        // scale down

        double max = Math.abs(powers[0]);
        for(int i = 1; i < 4; i++) {
            if(Math.abs(powers[i]) > max) {
                max = Math.abs(powers[i]);
            }
        }

        if(max > 1) {
            for(int i = 0; i < 4; i++) {
                powers[i] = powers[i]/max;
            }
        }

        return powers;
    }

    public static Pose2d clip(Pose2d pose, double max) {
        return new Pose2d(
                Range.clip(pose.getX(), -max, max),
                Range.clip(pose.getY(), -max, max),
                Range.clip(pose.getHeading(), -max, max)
        );
    }
}
