import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.BoxtubeKinematics;
import org.junit.*;

import java.util.Arrays;

public class JUnitTest {
    @Test
    public void testBlockMatrix() {
        doForwardKinematics(0, 0, 0, 0);
        

        Assert.assertEquals(4, 4);
    }

    public void doForwardKinematics(double pivotAngle, double extension, double armAngle, double wristAngle) {
        long startTime, endTime;

        System.out.println(Arrays.asList(pivotAngle, extension, armAngle, wristAngle));
        startTime = System.nanoTime();
        DMatrix3x3 res = BoxtubeKinematics.getBlockMatrix(pivotAngle, extension, armAngle, wristAngle);
        endTime = System.nanoTime();
        res.print();
        System.out.println("Angle: " + BoxtubeKinematics.getAngleFrom3x3(res));
        System.out.println("Time: " + (endTime - startTime) + " ns");
        System.out.println();
    }

    @Test
    public void testOriginToPivot2() {
        System.out.println("PI/2, 0, 0");
        long startTime;

        startTime = System.nanoTime();
        DMatrix3x3 res = BoxtubeKinematics.getPivot2(Math.PI/2,0, 0);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res));

        System.out.println();

        System.out.println("0, 0, PI/2");
        startTime = System.nanoTime();
        DMatrix3x3 res2 = BoxtubeKinematics.getPivot2(0,0, Math.PI/2);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res2.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res2));

        System.out.println();

        System.out.println("1, 5, 1");
        startTime = System.nanoTime();
        DMatrix3x3 res3 = BoxtubeKinematics.getPivot2(1,5, 1);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res3.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res3));

        System.out.println();

        System.out.println("0, 5, 2");
        startTime = System.nanoTime();
        DMatrix3x3 res4 = BoxtubeKinematics.getPivot2(0,5, 2);
        System.out.println("Time: " + (System.nanoTime() - startTime) + " ns");
        res4.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res4));

        System.out.println();

        Assert.assertEquals(4, 4);
    }

    @Test
    public void testOriginToRotatedPivot1() {
        DMatrix3x3 res = BoxtubeKinematics.getPivot1(1);
        res.print();
        System.out.println(BoxtubeKinematics.getAngleFrom3x3(res));
        Assert.assertEquals(4, 4);
    }

    @Test
    public void testRotation() {
        double theta = Math.PI/2;
        DMatrix3x3 R = new DMatrix3x3(Math.cos(theta), -Math.sin(theta), 0,
                Math.sin(theta), Math.cos(theta), 0,
                0, 0, 1);
        R.print();

        DMatrix3 v = new DMatrix3(1, 0, 1);
        v.print();
        DMatrix3 result = new DMatrix3();

        CommonOps_DDF3.mult(R, v, result);
        result.print();

        Assert.assertEquals(4, 4);
    }

    @Test
    public void testEJML() {
        DMatrix3x3 A = new DMatrix3x3(1,2,3,
                4,5,6,
                7,8,9);

        System.out.println(A);
        System.out.println("det: " + CommonOps_DDF3.det(A));
        Assert.assertEquals(4, 4);
    }

    @Test
    public void testCar() {
        Assert.assertEquals(1, 1);
    }
}
