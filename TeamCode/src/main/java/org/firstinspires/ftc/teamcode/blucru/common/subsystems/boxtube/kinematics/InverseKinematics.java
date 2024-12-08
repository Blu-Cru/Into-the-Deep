package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

public class InverseKinematics extends BoxtubeKinematics{
    public static DMatrix3x3 getEndEffectorMatrix(Pose2d endEffectorPose) {
        double cos = Math.cos(endEffectorPose.getHeading());
        double sin = Math.sin(endEffectorPose.getHeading());

        return new DMatrix3x3(
                cos, -sin, endEffectorPose.getX(),
                sin, cos, endEffectorPose.getY(),
                0, 0, 1
        );
    }

    public static DMatrix3x3 getP2(DMatrix3x3 endEffectorMatrix, double armAngle, double wristAngle) {
        DMatrix3x3 result = new DMatrix3x3();
        double cos = Math.cos(-armAngle);
        double sin = Math.sin(-armAngle);

        DMatrix3x3 endEffectorToP2 = new DMatrix3x3(
                cos, -sin, ARM_x + WRIST_x,
                sin, cos, ARM_y + Math.sin(wristAngle) * WRIST_y,
                0, 0, 1
        );

        CommonOps_DDF3.mult(endEffectorMatrix, endEffectorToP2, result);
        return result;
    }
}
