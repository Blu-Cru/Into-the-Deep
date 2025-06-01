package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class CollectRightBlockPath extends PIDPathBuilder {
    public CollectRightBlockPath() {
        super();
        this.setPower(0.7)
                .addMappedPoint(49, -33, 18, 4)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(3),
                        new ClampReleaseCommand(),
                        new WaitCommand(400),
                        new ExtensionMotionProfileCommand(10)
                ))
                .waitMillis(1300);
    }
}
