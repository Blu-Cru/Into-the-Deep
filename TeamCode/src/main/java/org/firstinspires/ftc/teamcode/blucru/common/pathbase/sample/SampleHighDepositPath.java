package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.RetractFromBasketCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleHighDepositPath extends PIDPathBuilder {
    public SampleHighDepositPath(boolean retractAfterDone) {
        super();
        this.setPower(0.25)
                .addMappedPoint(-56.1, -56.1, 45)
                .callback(() -> {
                    if(retractAfterDone) {
                        new SequentialCommandGroup(
                                new ClawOpenCommand(),
                                new WaitCommand(200),
                                new RetractFromBasketCommand()
                        ).schedule();
                    } else {
                        new SequentialCommandGroup(
                                new ClawOpenCommand()
//                                new WheelPowerCommand(-0.5),
//                                new WaitCommand(200),
//                                new ClampGrabCommand(),
//                                new WheelStopCommand()
                        ).schedule();
                    }
                })
                .waitMillis(250);
    }

    public SampleHighDepositPath() {
        this(true);
    }
}
