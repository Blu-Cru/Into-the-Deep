package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

public class InverseKinematics extends BoxtubeKinematics{
    public static double boxtubeOffsetInteriorAngle = Math.PI - Math.atan(BOXTUBE_y/BOXTUBE_x);

    public static double[] getJoints(Pose2d endEffectorPose, double wristAngle) {
        double xb = endEffectorPose.getX();
        double yb = endEffectorPose.getY();
        double thetab = endEffectorPose.getHeading();

        double bx = xb - PIVOT_x
                - (ARM_x + WRIST_x) * Math.cos(thetab)
                + (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.sin(thetab);
        double by = yb - PIVOT_y
                - (ARM_x + WRIST_x) * Math.sin(thetab)
                - (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.cos(thetab);

        System.out.println("bx: " + bx + " by: " + by);

        double triangleB = Math.sqrt(bx * bx + by * by);
        double triangleC = Math.sqrt(BOXTUBE_x * BOXTUBE_x + BOXTUBE_y * BOXTUBE_y);

        double discriminant = -triangleC * triangleC * Math.sin(boxtubeOffsetInteriorAngle) * Math.sin(boxtubeOffsetInteriorAngle) + triangleB * triangleB;
        double L = triangleC * Math.cos(boxtubeOffsetInteriorAngle) + Math.sqrt(discriminant);
//        System.out.println("L1: " + L1);
//        double L2 = triangleC * Math.cos(boxtubeOffsetInteriorAngle) - Math.sqrt(discriminant);
//        System.out.println("L2: " + L2);
//
//        double L;
//
//        if(L1 < 0 && L2 < 0) {
//            throw new IllegalArgumentException("No solution");
//        } else if (L1 < 0) {
//            L = L2;
//        } else if (L2 < 0) {
//            L = L1;
//        } else {
//            L = Math.min(L1, L2);
//        }

        Vector2d totalBoxtubeVector = new Vector2d(PIVOT_x + L, PIVOT_y);
        

        // TODO: debug angle
//        double angleB = Math.abs(Math.atan(by / bx));
        double angleB = Math.abs(Math.atan2(by, bx));
        double angleA = Math.asin(L * Math.sin(boxtubeOffsetInteriorAngle) / triangleB);
        // calculate pivot angle
        

        System.out.println("angleB: " + angleB + " angleA: " + angleA);

        double pivotAngle = angleB + Math.PI - angleA - boxtubeOffsetInteriorAngle;
        double armAngle = thetab - pivotAngle;

        return new double[] {pivotAngle, L, armAngle};
    }

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
