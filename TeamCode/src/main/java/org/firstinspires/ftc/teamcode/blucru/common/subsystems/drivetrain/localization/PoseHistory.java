package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.LinkedList;

// this class stores a history of poses and their timestamps for dead reckoning
// useful for computer vision, normally camera frames are processed from the past.
// atags and visionportal pipelines have methods to get time of frame
public class PoseHistory {
    static double STORAGE_NANOSECONDS = 1.0 * Math.pow(10.0, 9.0); // 1 second

    LinkedList<PoseMarker> poseList;

    /*
        Linked list of poses and their timestamps
        New poses are stored at the front of the linked list
        Timestamps are in nanoseconds

        The reason for using a linked list is because we need
        to add and remove elements from the front
        and back of the list, which is faster with a
        linked list than an arraylist
     */

    public PoseHistory() {
        // TODO: change to queue, remove until pose is found
        // initialize pose history
        poseList = new LinkedList<>();
    }

    public void add(Pose2d pose, Pose2d velocity) {
        poseList.addFirst(new PoseMarker(pose, velocity)); // add current pose to front of list

        // remove old poses
        long currentTime = System.nanoTime();
        while (poseList.size() > 0 && currentTime - poseList.getLast().nanoTime > STORAGE_NANOSECONDS) {
            poseList.removeLast(); // remove oldest pose from back of list until we have less than STORAGE_NANOSECONDS of poses
        }
    }

    public PoseMarker getPoseAtTime(long targetNanoTime) {
        PoseMarker poseMarkerAfterTime = poseList.get(0);
        PoseMarker poseMarkerBeforeTime = poseList.get(0);
        Log.v("PoseHistory", "Searching for pose at time " + targetNanoTime / Math.pow(10, 6));
        Log.v("PoseHistory", "Length of poseList: " + poseList.size());
        for(PoseMarker poseMarker : poseList) {
            if (poseMarker.nanoTime < targetNanoTime) {
                poseMarker.log("PoseMarker found");
//                Log.i("", "******************************************************************************************");
                poseMarkerBeforeTime = poseMarker;
                break;
            }
            else poseMarkerAfterTime = poseMarker;
            poseMarker.log("PoseMarker iterated");
        }

        Pose2d poseBefore = poseMarkerBeforeTime.pose;
        Pose2d poseAfter = poseMarkerAfterTime.pose;

        long timeBefore = targetNanoTime- poseMarkerBeforeTime.nanoTime;
//        System.out.println("before" + timeBefore);
        long timeAfter = poseMarkerAfterTime.nanoTime - targetNanoTime;
//        System.out.println("after" + timeAfter);
        long total = timeBefore + timeAfter;
//        System.out.println("total" + total);
//        Log.v("PoseHistory", "Before: " + poseBefore + " at time " + timeBefore / Math.pow(10, 6));
//        Log.v("PoseHistory", "After: " + poseAfter + " at time " + timeAfter / Math.pow(10, 6));

        double beforeMultiplier = (double) timeBefore / total;
//        System.out.println("before mult" + beforeMultiplier);
        double afterMultiplier = (double) timeAfter / total;
//        System.out.println("after mult" + afterMultiplier);

        Pose2d interpolatedPose = poseBefore.times(beforeMultiplier).plus(poseAfter.times(afterMultiplier)); // linear interpolation
//        Pose2d interpolatedPose = poseAfter;

        Log.v("PoseHistory", "Interpolated pose" + interpolatedPose);
        return new PoseMarker(interpolatedPose, poseMarkerBeforeTime.velocity);
    }

    public void offset(Pose2d poseDelta) {
        for (PoseMarker marker : poseList) {
            marker.pose = new Pose2d(marker.pose.vec().plus(poseDelta.vec()), marker.pose.getHeading());
        }
    }
}