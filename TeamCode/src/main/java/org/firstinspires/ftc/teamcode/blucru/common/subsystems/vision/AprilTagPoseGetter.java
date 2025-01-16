package org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;

@Config
public class AprilTagPoseGetter {
    public static double CAM_X = 0.0, CAM_Y = -7.074, CAM_HEADING = Math.toRadians(-90); // position of the camera relative to the center of the robot in inches
    static Pose2d CAM_POS = new Pose2d(CAM_X, CAM_Y, CAM_HEADING); // position of the camera relative to the center of the robot in inches
    static HashMap<Integer, Pose2d> TAGS = new HashMap<Integer, Pose2d>() {{
        put(11, new Pose2d(-72, 48, 0)); // tag 11
        put(12, new Pose2d(0, 72, -Math.PI/2)); // tag 12
        put(13, new Pose2d(72, 48, Math.PI)); // tag 13
        put(14, new Pose2d(72, -48, Math.PI)); // tag 14
        put(15, new Pose2d(0, -72, Math.PI/2)); // tag 15
        put(16, new Pose2d(-72, -48, 0)); // tag 16
    }};
    public static double MAX_UPDATE_DISTANCE = 72; // maximum update distance

    public static Vector2d getRobotToTagVector(double detectionX, double detectionY) {
        Vector2d camToTag = new Vector2d(detectionY, -detectionX);

        return camToTag.rotated(CAM_POS.getHeading()).plus(CAM_POS.vec());

//        //
//        double x = -detectionY + CAMERA_POS.getX();
//        double y = detectionX + CAMERA_POS.getY();
//        return new Vector2d(x, y);
    }

    public static Vector2d getTagToRobotVector(Vector2d robotToTag, double detectionYawRad) {
        return robotToTag.rotated(-detectionYawRad);
    }

    public static Pose2d getRobotPose(AprilTagDetection detection) {
        Vector2d robotToTag = getRobotToTagVector(detection.ftcPose.x, detection.ftcPose.y);
        Vector2d tagToRobot = getTagToRobotVector(robotToTag, Math.toRadians(detection.ftcPose.yaw));
        Pose2d tagPose = TAGS.get(detection.id);

        return new Pose2d(tagPose.vec().plus(tagToRobot), tagPose.getHeading() - Math.toRadians(detection.ftcPose.yaw));
    }

    public static Pose2d getRobotPoseWithHeading(AprilTagDetection detection, double heading) {
        Vector2d robotToTag = getRobotToTagVector(detection.ftcPose.x, detection.ftcPose.y);
        Vector2d globalTagToRobot = robotToTag.rotated(heading).unaryMinus();
        Pose2d tagPose = TAGS.get(detection.id);

        return new Pose2d(tagPose.vec().plus(globalTagToRobot), heading);
    }

    // update without robot heading (not good)
    public static Pose2d getRobotPoseAtTimeOfFrame(List<AprilTagDetection> detections) {
        if(detections.size() == 0) {
            throw new NullPointerException("No tags found");
        } else {
            AprilTagDetection closestDetection = detections.get(0);
            double closestDistance = Math.hypot(closestDetection.ftcPose.x, closestDetection.ftcPose.y);

            for (AprilTagDetection detection : detections) {
                double distance = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
                if(distance < closestDistance) {
                    closestDetection = detection;
                    closestDistance = distance;
                }
            }

            if(closestDistance > MAX_UPDATE_DISTANCE) {
                throw new IllegalStateException("Too far away to update tags");
            }

            return getRobotPose(closestDetection);
        }
    }

    // update with robot heading (good)
    public static Pose2d getRobotPoseAtTimeOfFrame(List<AprilTagDetection> detections, double heading) {
        if(detections.size() == 0) {
            Log.e("TagPoseGetter", "No tags found");
            return null;
        } else {
            AprilTagDetection closestDetection = detections.get(0);
            double closestDistance = Math.hypot(closestDetection.ftcPose.x, closestDetection.ftcPose.y);

            for (AprilTagDetection detection : detections) {
                double distance = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
                if(distance < closestDistance) {
                    closestDetection = detection;
                    closestDistance = distance;
                }
            }

            if(closestDistance > MAX_UPDATE_DISTANCE) {
                Log.e("TagPoseGetter", "Too far away to update tags");
                return null;
            }

            Pose2d poseWithHeading = getRobotPoseWithHeading(closestDetection, heading);

            Log.i("TagPoseGetter", "got pose: " + poseWithHeading + " using tag # " + closestDetection.id);
            return poseWithHeading;
        }
    }

    public static void updateCamPose() {
        CAM_POS = new Pose2d(CAM_X, CAM_Y, CAM_HEADING);
    }
}
