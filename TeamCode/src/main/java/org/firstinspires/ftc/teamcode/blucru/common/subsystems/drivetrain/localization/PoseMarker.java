package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

// this class is used for a single marker with pose, velocity, and timestamp for the PoseHistory class
public class PoseMarker {
    long nanoTime;
    public Pose2d pose, velocity;

    public PoseMarker(Pose2d pose, Pose2d velocity) {
//        Log.i("", "******************************************************************************************");
        nanoTime = System.nanoTime();
        this.pose = new Pose2d(new Vector2d(pose.getX(), pose.getY()), pose.getHeading());
        this.velocity = new Pose2d(new Vector2d(velocity.getX(), velocity.getY()), velocity.getHeading());
//        this.log("PoseMarker created");
    }

    public PoseMarker(long nanoTime, Pose2d pose) {
        this.nanoTime = nanoTime;
        this.pose = new Pose2d(new Vector2d(pose.getX(), pose.getY()), pose.getHeading());
    }

    public void log(String tag) {
        Log.v(tag,  "PoseMarker at pose: " + pose + ", Pose hash code:" + pose.hashCode() + ", Velocity: " + velocity + ", Time: " + nanoTime / Math.pow(10.0, 6.0));
    }

    public void logLine() {
        Log.v("PoseMarker", "******************************************************************************************");
    }
}
