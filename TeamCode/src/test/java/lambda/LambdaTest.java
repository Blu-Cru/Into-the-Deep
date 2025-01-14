package lambda;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path.PathSegment;
import org.junit.Assert;
import org.junit.Test;

public class LambdaTest {
    @Test
    public void testLambda() throws InterruptedException {
        boolean running = true;
        LambdaTestPathSegment segment = new LambdaTestPathSegment();

        Path path = new PIDPathBuilder()
                .addSegment(segment)
                .schedule(new LambdaTestCommand())
                .callback(() -> System.out.println("running callback at time: " + System.currentTimeMillis()))
                .build();

        double startTime = System.currentTimeMillis();

        while (!path.isDone()) {
            if(System.currentTimeMillis() - startTime > 1000) {
                segment.done = true;
            }

            path.run();
            CommandScheduler.getInstance().run();
        }
        Assert.assertTrue(true);
    }
}
