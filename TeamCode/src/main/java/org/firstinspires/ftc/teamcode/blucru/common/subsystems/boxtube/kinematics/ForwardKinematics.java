package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

public class ForwardKinematics extends BoxtubeKinematics {
    public static DMatrix3x3 getForwardKinematics(double pivotAngle, double extension, double armAngle, double wristAngle) {
        DMatrix3x3 result = new DMatrix3x3();

        DMatrix3x3 P2toBlock = new DMatrix3x3(
                1, 0, ARM_x + WRIST_x,
                0, 1, ARM_y + Math.sin(wristAngle) * WRIST_y,
                0, 0, 1
        );

        DMatrix3x3 pivot2 = getPivot2(pivotAngle, extension, armAngle);

        System.out.println("Pivot2" + getPoseFrom3x3(pivot2).minus(new Pose2d(PIVOT_x, PIVOT_y, 0)));

        CommonOps_DDF3.mult(pivot2, P2toBlock, result);
        return result;
    }

    public static DMatrix3x3 getPivot2(double pivotAngle, double extension, double armAngle) {
        DMatrix3x3 result = new DMatrix3x3();

        double cos = Math.cos(armAngle);
        double sin = Math.sin(armAngle);
        DMatrix3x3 extensionMatrix = new DMatrix3x3(
                cos, -sin, BOXTUBE_x + extension,
                sin, cos, BOXTUBE_y,
                0, 0, 1
        );

        DMatrix3x3 p1 = getPivot1(pivotAngle);

        CommonOps_DDF3.mult(p1, extensionMatrix, result);
        return result;
    }

    public static DMatrix3x3 getPivot1(double pivotAngle){
        double cos = Math.cos(pivotAngle);
        double sin = Math.sin(pivotAngle);

        return new DMatrix3x3(
                cos, -sin, PIVOT_x,
                sin, cos, PIVOT_y,
                0, 0, 1
        );
    }
}
