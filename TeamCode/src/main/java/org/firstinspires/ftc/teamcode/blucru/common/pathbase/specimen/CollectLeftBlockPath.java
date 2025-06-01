package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class CollectLeftBlockPath extends PIDPathBuilder {
    public CollectLeftBlockPath() {
        super();
        this.setPower(0.5)
                .addMappedPoint(20, -44, 45, 2)
                .schedule(new SequentialCommandGroup(
                        new PivotRetractCommand(),
                        new WaitCommand(200),
                        new ExtensionCommand(2)
                ))
                .addMappedPoint(29, -31.5, 14,3)
                .schedule(new SequentialCommandGroup(
                        new ClawOpenCommand(),
                        new WaitCommand(100),
                        new ExtensionMotionProfileCommand(10)
                ))
                .waitMillis(1000);
    }
}
