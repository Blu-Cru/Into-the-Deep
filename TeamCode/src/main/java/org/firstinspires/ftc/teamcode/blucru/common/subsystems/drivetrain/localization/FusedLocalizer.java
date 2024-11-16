package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision.AprilTagPoseGetter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

// this class combines odometry, IMU, and AprilTags with weighted updates
public class FusedLocalizer {
    public static double TAG_UPDATE_DELAY = 100; // ms between tag updates
    Localizer deadWheels;
    PoseHistory poseHistory;
    long lastFrameTime;
    double lastTagUpdateMillis;

    public FusedLocalizer(Localizer localizer) {
        deadWheels = localizer;
        poseHistory = new PoseHistory();

        lastFrameTime = System.nanoTime();
        lastTagUpdateMillis = System.currentTimeMillis();
    }

    public void update() {
        // make a copy of the current pose, so that the pose history doesn't get updated with the same object
        deadWheels.update();
        Pose2d currentPose = deadWheels.getPoseEstimate();
        //Log.v("Marker Entry", "Pos" + currentPose);
        poseHistory.add(currentPose, deadWheels.getPoseVelocity());

    }

    public boolean updateAprilTags(AprilTagProcessor tagProcessor) {
        if(System.currentTimeMillis() - lastTagUpdateMillis < TAG_UPDATE_DELAY) return false; // only update every TAG_UPDATE_DELAY ms

        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections.size() < 1) {
            Log.v("FusedLocalizer", "No tags found");
            return false;
        }

        // get odo pose at the time of the tag pose
        long timeOfFrame = detections.get(0).frameAcquisitionNanoTime;
        if(timeOfFrame==lastFrameTime) {
            Log.i("FusedLocalizer", "Already updated with this frame");
            return false;
        }
        PoseMarker poseMarkerAtFrame = poseHistory.getPoseAtTime(timeOfFrame);
        Pose2d poseAtFrame = poseMarkerAtFrame.pose;
        Pose2d velocityAtFrame = poseMarkerAtFrame.velocity;

        long timeSinceFrame = System.nanoTime() - timeOfFrame;
        Log.v("FusedLocalizer", "Time since frame:" + timeSinceFrame);

        // save reference to tag pose
        Pose2d tagPose;
        try {
            tagPose = AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(detections, poseAtFrame.getHeading());
        } catch(Exception e) {
            return false;
        }

        double weight = getWeight(velocityAtFrame);
        Log.d("FusedLocalizer", "Velocity at frame: " + velocityAtFrame);
        Log.d("FusedLocalizer", "Weight: " + weight);

        if(weight == 0) {
            Log.e("FusedLocalizer", "Weight is 0, not updating");
            return false;
        }

        // calculate change from old odo pose to current pose
        Pose2d currentPose = deadWheels.getPoseEstimate();

        Pose2d odoPoseError = tagPose.minus(poseAtFrame);
        Pose2d weightedCorrection = odoPoseError.times(weight);

        Log.d("FusedLocalizer", "Tag pose: " + tagPose);
        Log.d("FusedLocalizer", "Pose at frame:" + poseAtFrame);
        Log.d("FusedLocalizer", "Current pose: " + currentPose);
        Log.d("FusedLocalizer", "Raw correction: " + odoPoseError);
        Log.d("FusedLocalizer", "Weighted correction: " + weightedCorrection);

        Pose2d newPose = new Pose2d(currentPose.vec().plus(weightedCorrection.vec()), currentPose.getHeading());
        Log.i("FusedLocalizer", "Updated pose to: " + newPose);

        // set pose estimate to tag pose + delta
        deadWheels.setPoseEstimate(newPose);
        deadWheels.update();
        lastTagUpdateMillis = System.currentTimeMillis();
        // add tag - odo to pose history
        poseHistory.offset(weightedCorrection);
        lastFrameTime = timeOfFrame;
        return true;
    }

    public double getWeight(Pose2d velocity) {
        double angVel = velocity.getHeading();
        double vel = velocity.vec().norm();

        double totalVel = Math.hypot(vel, angVel * 12);

        // this function determines the weight of the update based on the velocity of the robot
        // put it into desmos to visualize
        return Range.clip(-0.75*Math.atan(.07 * totalVel-5), 0, 1);
    }
}
