package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class CollectLeftBlockPath extends PIDPathBuilder {
    public CollectLeftBlockPath() {
        super();
        this.setPower(0.5)
                .addMappedPoint(20, -44, 45, 4)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(4),
                        new ArmDropToGroundCommand()
                ))
                .addMappedPoint(28, -36, 23,3)
                .schedule(new SequentialCommandGroup(
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand(),
                        new ExtensionMotionProfileCommand(8)
                ))
                .waitMillis(1000);
    }
}
