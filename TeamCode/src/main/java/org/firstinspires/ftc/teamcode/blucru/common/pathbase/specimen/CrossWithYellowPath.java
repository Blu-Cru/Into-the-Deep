package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
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
                .addMappedPoint(-30, -55, 0, 20)
                .schedule(new PivotCommand(Math.PI/2))
                .setPower(0.75)
                .addMappedPoint(-30, -58, 80);
    }
}
