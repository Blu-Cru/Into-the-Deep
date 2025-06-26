package kinematics;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.junit.Test;

import java.util.Arrays;

public class TurretInverseKinematicsTest {

    @Test
    public void testKinematics(){
        doInverseKinematics(new Vector2d(16,0));
        doInverseKinematics(new Vector2d(16,2));
        doInverseKinematics(new Vector2d(20, 5));
        doInverseKinematics(new Vector2d(15, -3));
    }

    public void doInverseKinematics(Vector2d point){
        double[] res = BoxtubeKinematics.getExtensionTurretPose(point);
        System.out.println(Arrays.toString(res));
    }


}
