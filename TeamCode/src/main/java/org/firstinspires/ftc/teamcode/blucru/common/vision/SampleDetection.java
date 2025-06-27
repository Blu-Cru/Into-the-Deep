package org.firstinspires.ftc.teamcode.blucru.common.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.util.Sample;

public class SampleDetection {
    public Pose2d globalPose;
    public Sample color;
    public double distance;

    public SampleDetection (Pose2d globalPose, Sample color, double distance) {
        this.globalPose = globalPose;
        this.color = color;
        this.distance = distance;
    }
}
