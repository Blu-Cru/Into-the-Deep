package org.firstinspires.ftc.teamcode.blucru.common.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.util.Sample;

public class SampleDetection {
    public Pose2d globalPose;
    public Sample color;
    public double distance;
    public double robotDetectionOrientation;

    public SampleDetection (Pose2d globalPose, Sample color, double distance, double robotDetectionOrientation) {
        this.globalPose = globalPose;
        this.color = color;
        this.distance = distance;
        this.robotDetectionOrientation = robotDetectionOrientation;
    }

    public double getPenalty() {
        // optimal detection is pi/2, cosine is good for finding the penalty from vertical
        double anglePenalty = 10.0 * Math.abs(Math.sin(robotDetectionOrientation));

        return anglePenalty + distance;
    }
}
