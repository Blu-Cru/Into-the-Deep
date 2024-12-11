package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BoxtubeIKPose {
    public Pose2d pose;
    public double wristAngle;
    public double armAngle;
    public double pivotAngle;
    public double extensionLength;

    public BoxtubeIKPose(Pose2d pose, double wristAngle) {
        this.pose = pose;
        this.wristAngle = wristAngle;

        double[] joints = BoxtubeInverseKinematics.getJoints(pose, wristAngle);

        this.pivotAngle = joints[0];
        this.extensionLength = joints[1];
        this.armAngle = joints[2];
    }
}
