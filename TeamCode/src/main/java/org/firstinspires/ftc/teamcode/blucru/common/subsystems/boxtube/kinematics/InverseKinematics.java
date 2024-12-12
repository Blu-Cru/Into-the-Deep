package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

public class InverseKinematics extends BoxtubeKinematics{
    public static double boxtubeOffsetInteriorAngle = Math.PI - Math.atan(BOXTUBE_y/BOXTUBE_x);

    public static double[][] getJointPositionsVelocities(Pose2d endEffectorPose, Vector2d endEffectorVelocity, double wristAngle) {
        double xb = endEffectorPose.getX();
        double yb = endEffectorPose.getY();
        double thetab = endEffectorPose.getHeading();

        double xbDot = endEffectorVelocity.getX();
        double ybDot = endEffectorVelocity.getY();

        // b is the offset the boxtube needs to reach
        double bx = xb - PIVOT_x
                - (ARM_x + WRIST_x) * Math.cos(thetab)
                + (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.sin(thetab);
        double by = yb - PIVOT_y
                - (ARM_x + WRIST_x) * Math.sin(thetab)
                - (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.cos(thetab);

        System.out.println("bx: " + bx + " by: " + by);

        // use law of cosines to find L

        double triangleB = Math.sqrt(bx * bx + by * by);
        double triangleC = Math.sqrt(BOXTUBE_x * BOXTUBE_x + BOXTUBE_y * BOXTUBE_y);

        double sinInterior = Math.sin(boxtubeOffsetInteriorAngle);
        double discriminant = -triangleC * triangleC * sinInterior * sinInterior + triangleB * triangleB;
        double L = triangleC * Math.cos(boxtubeOffsetInteriorAngle) + Math.sqrt(discriminant);

        // angle of boxtube (offset and extension) from x-axis
        double boxtubeAngle = Angle.normDelta(Math.atan2(BOXTUBE_y, BOXTUBE_x + L));
        // angle of b, target point for boxtube to reach from x axis
        double bAngle = Angle.normDelta(Math.atan2(by, bx));

        double pivotAngle = bAngle - boxtubeAngle;
        double armAngle = thetab - pivotAngle;

        double[] positions = new double[] {pivotAngle, L, armAngle};

        double Ldot = (bx * xbDot + by * ybDot) / Math.sqrt(BOXTUBE_x * BOXTUBE_x + BOXTUBE_y * BOXTUBE_y);
        double pivotDot = (ybDot * bx - xbDot * by) / (bx * bx + by * by);

        double[] velocities = new double[] {pivotDot, Ldot};

        return new double[][] {positions, velocities};
    }

    public static double[] getJoints(Pose2d endEffectorPose, double wristAngle) {
        double xb = endEffectorPose.getX();
        double yb = endEffectorPose.getY();
        double thetab = endEffectorPose.getHeading();

        // b is the offset the boxtube needs to reach
        double bx = xb - PIVOT_x
                - (ARM_x + WRIST_x) * Math.cos(thetab)
                + (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.sin(thetab);
        double by = yb - PIVOT_y
                - (ARM_x + WRIST_x) * Math.sin(thetab)
                - (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.cos(thetab);

        System.out.println("bx: " + bx + " by: " + by);

        // use law of cosines to find L

        double triangleB = Math.sqrt(bx * bx + by * by);
        double triangleC = Math.sqrt(BOXTUBE_x * BOXTUBE_x + BOXTUBE_y * BOXTUBE_y);

        double sinInterior = Math.sin(boxtubeOffsetInteriorAngle);
        double discriminant = -triangleC * triangleC * sinInterior * sinInterior + triangleB * triangleB;
        double L = triangleC * Math.cos(boxtubeOffsetInteriorAngle) + Math.sqrt(discriminant);

        // angle of boxtube (offset and extension) from x-axis
        double boxtubeAngle = Angle.normDelta(Math.atan2(BOXTUBE_y, BOXTUBE_x + L));
        // angle of b, target point for boxtube to reach from x axis
        double bAngle = Angle.normDelta(Math.atan2(by, bx));

        double pivotAngle = bAngle - boxtubeAngle;
        double armAngle = thetab - pivotAngle;

        return new double[] {pivotAngle, L, armAngle};
    }

    // get velocity of pivot and extension, arm and wrist are constant
    public static double[] getPivotExtensionVelocities(Vector2d endEffectorVelocity, Pose2d endEffectorPosition, double wristAngle) {
        double xb = endEffectorPosition.getX();
        double yb = endEffectorPosition.getY();
        double thetab = endEffectorPosition.getHeading();

        double xbDot = endEffectorVelocity.getX();
        double ybDot = endEffectorVelocity.getY();

        double bx = xb - PIVOT_x
                - (ARM_x + WRIST_x) * Math.cos(thetab)
                + (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.sin(thetab);
        double by = yb - PIVOT_y
                - (ARM_x + WRIST_x) * Math.sin(thetab)
                - (ARM_y + WRIST_y * Math.sin(wristAngle)) * Math.cos(thetab);

        double Ldot = (bx * xbDot + by * ybDot) / Math.sqrt(BOXTUBE_x * BOXTUBE_x + BOXTUBE_y * BOXTUBE_y);
        double pivotDot = (ybDot * bx - xbDot * by) / (bx * bx + by * by);

        return new double[] {pivotDot, Ldot};
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
