package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeInverseKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;

public class BoxtubeIKPoseVelocity {
    public Pose2d blockPose;
    public Vector2d blockVel;
    public double wristAngle;
    public double armAngle;
    public Vector2d pivotState;
    public Vector2d extensionState;

    public BoxtubeIKPoseVelocity(BoxtubeSpline spline) {
        if(spline.started) {
            blockPose = new Pose2d(spline.getPosition(), spline.blockAngleProfile.getInstantTargetPosition());
            blockVel = spline.getVelocity();
            wristAngle = spline.wristAngleProfile.getInstantTargetPosition();
        } else {
            blockPose = spline.startPose;
            blockVel = spline.startTangent.div(spline.duration);
            wristAngle = Robot.getInstance().wrist.getAngle();
        }

        double[][] poseVelocities = BoxtubeInverseKinematics.getJointPositionsVelocities(blockPose, blockVel, wristAngle);
        pivotState = new Vector2d(poseVelocities[0][0], poseVelocities[1][0]);
        extensionState = new Vector2d(poseVelocities[0][1], poseVelocities[1][1]);
        armAngle = poseVelocities[0][2];
    }
}
