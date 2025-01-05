package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpitPath extends PIDPathBuilder {
    public SpitPath() {
        super();
        this.setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(4),
                        new ArmPreIntakeCommand()
                ))
                .addMappedPoint(38, -42, -60, 7)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(11),
                        new ArmDropToGroundCommand(),
                        new WaitCommand(150),
                        new WheelReverseCommand(),
                        new ClampReleaseCommand(),
                        new WaitCommand(200),
                        new ArmPreIntakeCommand(),
                        new WheelStopCommand(),
                        new ClampGrabCommand(),
                        new ExtensionCommand(4)
                ))
                .waitMillis(300);
    }
}
