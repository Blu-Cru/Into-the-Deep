package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class CollectCenterBlockPath extends PIDPathBuilder {
    public CollectCenterBlockPath() {
        super();
        this.setPower(0.7)
                .addMappedPoint(39, -33, 18, 4)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(1),
                        new ClawReleaseCommand(),
                        new WaitCommand(330),
                        new ExtensionMotionProfileCommand(10)
                ))
                .waitMillis(1300);
    }
}
