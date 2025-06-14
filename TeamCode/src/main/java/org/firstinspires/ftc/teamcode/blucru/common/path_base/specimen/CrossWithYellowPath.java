package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class CrossWithYellowPath extends PIDPathBuilder {
    public CrossWithYellowPath() {
        super();
        this.setPower(1.0)
                .schedule(
//                        new SequentialCommandGroup(
//                                new ClampGrabCommand(),
//                                new WheelStopCommand(),
//                                new WaitCommand(300),
                                new FullRetractCommand()
//                        )
                )
                .addMappedPoint(-30, -53, 0, 20)
                .schedule(new PivotCommand(Math.PI/2))
                .setPower(0.75)
                .addMappedPoint(-30, -58, 80);
    }
}
