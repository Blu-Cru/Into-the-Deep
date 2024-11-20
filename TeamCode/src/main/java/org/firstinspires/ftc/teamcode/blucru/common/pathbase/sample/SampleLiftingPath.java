package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BasketBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleLiftingPath extends PIDPathBuilder {
    public SampleLiftingPath(long waitBeforeLiftMillis) {
        super();
        this.setPower(0.3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(waitBeforeLiftMillis),
                        new BasketBackHighCommand()
                ))
                .addMappedPoint(-52, -52, 45, 5)
                .waitMillis(400);
    }
}
