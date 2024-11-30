package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleLiftingPath extends PIDPathBuilder {
    public SampleLiftingPath(boolean useSubsystems, long waitBeforeLiftMillis) {
        super();
        this.setPower(0.3);

        if(useSubsystems) {
            this.schedule(new SequentialCommandGroup(
                    new WaitCommand(waitBeforeLiftMillis),
                    new SampleBackHighCommand()
            ));
        }
        this.addMappedPoint(-53, -53, 45, 5)
                .waitMillis(400);
    }

    public SampleLiftingPath(long waitBeforeLiftMillis) {
        this(true, waitBeforeLiftMillis);
    }

    public SampleLiftingPath() {
        this(true,0);
    }

    public SampleLiftingPath(boolean useSubsystems) {
        this(useSubsystems,0);
    }
}
