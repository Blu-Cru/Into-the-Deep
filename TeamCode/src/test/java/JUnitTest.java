import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.junit.*;

public class JUnitTest {
    @Test
    public void testCar() {
        Assert.assertEquals(1, 1);
    }

    @Test
    public void testEJML() {
        DMatrix3x3 A = new DMatrix3x3(1,2,3,
                                       4,5,6,
                                       7,8,9);
//        SimpleMatrix A = new SimpleMatrix(2, 2);
        System.out.println(A);
        System.out.println("det: " + CommonOps_DDF3.det(A));
        Assert.assertEquals(4, 4);
    }
}
