package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

public class BoxtubeKinematics {
    public static final double
            BOXTUBE_x = 11.79, BOXTUBE_y = -1.058,
            ARM_x = 3.0, ARM_y = 0.5,
            WRIST_x = 2.5, WRIST_y = -0.5,

            Px = -5.5, Py = 4.0; // everything in inches/radians

    public static DMatrix3x3 PIVOT1 = new DMatrix3x3(
            1, 0, Px,
            0, 1, Py,
            0, 0, 1
    );

    public static DMatrix3x3 getBlockMatrix(double pivotAngle, double extension, double armAngle, double wristAngle) {
        DMatrix3x3 result = new DMatrix3x3();

        DMatrix3x3 transform = new DMatrix3x3(
                1, 0, ARM_x,
                0, 1, ARM_y + Math.sin(wristAngle) * WRIST_y,
                0, 0, 1
        );

        DMatrix3x3 pivot2 = getPivot2(pivotAngle, extension, armAngle);

        CommonOps_DDF3.mult(pivot2, transform, result);
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
        DMatrix3x3 result = new DMatrix3x3();
        double cos = Math.cos(pivotAngle);
        double sin = Math.sin(pivotAngle);

        DMatrix3x3 rotation = new DMatrix3x3(
                cos, -sin, 0,
                sin, cos, 0,
                0, 0, 1
        );

        CommonOps_DDF3.mult(PIVOT1, rotation, result);
        return result;
    }

    public static double getAngleFrom3x3 (DMatrix3x3 matrix) {
        return Math.atan2(matrix.get(1, 0), matrix.get(0, 0));
    }

    public static Vector2d getVectorFrom3x3 (DMatrix3x3 matrix) {
        return new Vector2d(matrix.get(0, 2), matrix.get(1, 2));
    }
}
