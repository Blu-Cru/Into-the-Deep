package kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrix3x3;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeForwardKinematics;
import org.junit.Assert;
import org.junit.Test;

import java.util.Arrays;

public class BoxtubeForwardKinematicsTest {
    @Test
    public void testForwardKinematics() {
        doForwardKinematics(1.35, 8.5, -0.3-1.35, -Math.PI);
        doForwardKinematics(0.27, 8, -0.27, 0);
        doForwardKinematics(1.45, 10, Math.PI-1.45, -Math.PI);
        doForwardKinematics(1.45, 3, Math.PI-1.45, -Math.PI);
//        doForwardKinematics(0, 0, 0, 0);
//        doForwardKinematics(0, 0, 0, -Math.PI/2);
//        doForwardKinematics(0, 0, Math.PI/2, 0);
//        doForwardKinematics(0, 0, Math.PI/2, -Math.PI/2);
//        doForwardKinematics(0, 5, 0, 0);
//        doForwardKinematics(0, 5, 0, -Math.PI/2);
//        doForwardKinematics(Math.PI/2, 0, 0, 0);
//        doForwardKinematics(Math.PI/2, 0, 0, -Math.PI/2);
//        doForwardKinematics(Math.PI/2, 0, Math.PI/2, 0);
//        doForwardKinematics(Math.PI/2, 0, Math.PI/2, -Math.PI/2);
//        doForwardKinematics(Math.PI/2, 5, 0, 0);

        Assert.assertEquals(4, 4);
    }

    public void doForwardKinematics(double pivotAngle, double extension, double armAngle, double wristAngle) {
        long startTime, endTime;

        System.out.println("FORWARD KINEMATICS\nPivot: " + pivotAngle + "  Extension: " + extension + "  Arm Angle: " + armAngle + "  Wrist Angle: " + wristAngle);
//        System.out.println(Arrays.asList(pivotAngle, extension, armAngle, wristAngle));
        startTime = System.nanoTime();
        Pose2d res = BoxtubeForwardKinematics.getPoseFrom3x3(BoxtubeForwardKinematics.getEndEffectorMatrix(pivotAngle, extension, armAngle, wristAngle));
        endTime = System.nanoTime();
        System.out.println("\nBLOCK POSE: " + res);
        System.out.println("Time: " + (endTime - startTime) + " ns");
        System.out.println();
    }

    @Test
    public void testOriginToPivot2() {
        System.out.println("PI/2, 0, 0");
        long startTime;

        startTime = System.nanoTime();
        DMatrix3x3 res = BoxtubeForwardKinematics.getPivot2(Math.PI/2,0, 0);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res));

        System.out.println();

        System.out.println("0, 0, PI/2");
        startTime = System.nanoTime();
        DMatrix3x3 res2 = BoxtubeForwardKinematics.getPivot2(0,0, Math.PI/2);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res2.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res2));

        System.out.println();

        System.out.println("1, 5, 1");
        startTime = System.nanoTime();
        DMatrix3x3 res3 = BoxtubeForwardKinematics.getPivot2(1,5, 1);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res3.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res3));

        System.out.println();

        System.out.println("0, 5, 2");
        startTime = System.nanoTime();
        DMatrix3x3 res4 = BoxtubeForwardKinematics.getPivot2(0,5, 2);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res4.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res4));

        System.out.println();

        Assert.assertEquals(4, 4);
    }

    @Test
    public void testOriginToRotatedPivot1() {
        DMatrix3x3 res = BoxtubeForwardKinematics.getPivot1(1);
        res.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res));
        Assert.assertEquals(4, 4);
    }
}
