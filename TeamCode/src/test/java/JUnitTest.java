import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.firstinspires.ftc.teamcode.blucru.common.vision.SampleDetectionProcessor;
import org.jetbrains.annotations.TestOnly;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

public class JUnitTest {
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
