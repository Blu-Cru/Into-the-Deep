package lambda;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.PathSegment;

public class LambdaTestPathSegment implements PathSegment {
    public boolean done;
    public LambdaTestPathSegment() {
        done = false;
    }

    @Override
    public boolean isDone() {
        return done;
    }

    @Override
    public void start() {

    }

    @Override
    public boolean failed() {
        return false;
    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public void run() {

    }
}
