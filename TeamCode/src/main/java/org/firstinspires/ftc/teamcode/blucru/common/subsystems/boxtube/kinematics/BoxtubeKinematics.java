package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.ejml.data.DMatrix3x3;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;

@Config
public class BoxtubeKinematics {
    public static double
            BOXTUBE_x = 12.15, BOXTUBE_y = -1.05866,
            ARM_x = 2.948819, ARM_y = -1.035,
            WRIST_x = 3.8448819, WRIST_y = 1.3759843,

            ARM_LENGTH = 6.743,
            BOXTUBE_RETRACT_OFFSET = 7.66,

            PIVOT_x = -5.354, PIVOT_y = 5.734; // everything in inches/radians


    public static Pose2d getPoseFrom3x3 (DMatrix3x3 matrix) {
        return new Pose2d(matrix.get(0, 2), matrix.get(1, 2), getAngleFrom3x3(matrix));
    }

    public static double getAngleFrom3x3 (DMatrix3x3 matrix) {
        return Math.atan2(matrix.get(1, 0), matrix.get(0, 0));
    }

    public static Vector2d getVectorFrom3x3 (DMatrix3x3 matrix) {
        return new Vector2d(matrix.get(0, 2), matrix.get(1, 2));
    }

    public static double[] getExtensionTurretPose (Point2d targetPoint) {
        double aSinArg = targetPoint.y / ARM_LENGTH;
        double turretAngle;

        if (aSinArg >= 1.0)
            turretAngle = Math.PI/2;
        else if (aSinArg <= -1.0)
            turretAngle = -Math.PI/2;
        else
            turretAngle = Math.asin(targetPoint.y / ARM_LENGTH);

        double extensionDistance = targetPoint.x - BOXTUBE_RETRACT_OFFSET - Math.cos(turretAngle);

        return new double[]{extensionDistance, turretAngle};
    }
}
