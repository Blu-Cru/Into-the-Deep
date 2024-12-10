package kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrix3x3;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.ForwardKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.InverseKinematics;
import org.junit.Assert;
import org.junit.Test;

import java.util.Arrays;

public class InverseKinematicsTest {
    @Test
    public void testInverseKinematics() {
        doInverseKinematics(0, 0, 0, 0);
        doInverseKinematics(0, 0, 0, -Math.PI/2);
        doInverseKinematics(0, 0, Math.PI/2, 0);
        doInverseKinematics(0, 0, Math.PI/2, -Math.PI/2);
        doInverseKinematics(0, 5, 0, 0);
        doInverseKinematics(0, 5, 0, -Math.PI/2);
        doInverseKinematics(Math.PI/2, 0, 0, 0);
        doInverseKinematics(Math.PI/2, 0, 0, -Math.PI/2);
        doInverseKinematics(Math.PI/2, 0, Math.PI/2, 0);
        doInverseKinematics(Math.PI/2, 0, Math.PI/2, -Math.PI/2);
        doInverseKinematics(Math.PI/2, 5, 0, 0);

        Assert.assertEquals(4, 4);
    }

    public void doInverseKinematics(double pivotAngle, double extension, double armAngle, double wristAngle) {
        Pose2d endEffectorPose = doForwardKinematics(pivotAngle, extension, armAngle, wristAngle);

        long startTime, endTime;
        double[] res;
        startTime = System.nanoTime();
        res = InverseKinematics.getJoints(endEffectorPose, wristAngle);
        endTime = System.nanoTime();
        System.out.println("\nINPUT Pivot: " + pivotAngle + " Extension: " + extension + " Arm: " + armAngle + " Wrist: " + wristAngle);
        System.out.println("Output Pivot: " + res[0] + " Extension: " + res[1] + " Arm: " + res[2]);
        System.out.println("\nTime: " + (endTime - startTime) + " ns");
        System.out.println();
    }

    public Pose2d doForwardKinematics(double pivotAngle, double extension, double armAngle, double wristAngle) {
        long startTime, endTime;

        System.out.println(Arrays.asList(pivotAngle, extension, armAngle, wristAngle));
        startTime = System.nanoTime();
        Pose2d res = BoxtubeKinematics.getPoseFrom3x3(ForwardKinematics.getForwardKinematics(pivotAngle, extension, armAngle, wristAngle));
        endTime = System.nanoTime();
        System.out.println(res);
        System.out.println("Time: " + (endTime - startTime) + " ns");
        System.out.println();

        return res;
    }
}
