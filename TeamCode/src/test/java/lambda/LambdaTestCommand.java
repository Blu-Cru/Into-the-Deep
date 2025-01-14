package lambda;

import com.arcrobotics.ftclib.command.InstantCommand;

public class LambdaTestCommand extends InstantCommand {
    public LambdaTestCommand() {
        super(() -> {
            System.out.println("running lambda at time: " + System.currentTimeMillis());
        });
        System.out.println("instantiated lambda at time: " + System.currentTimeMillis());
    }
}
