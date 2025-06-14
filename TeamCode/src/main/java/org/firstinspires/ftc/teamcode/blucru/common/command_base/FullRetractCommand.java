package org.firstinspires.ftc.teamcode.blucru.common.command_base;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;

public class FullRetractCommand extends SequentialCommandGroup {
    public FullRetractCommand() {
        super(
                new EndEffectorRetractCommand(),
                new BoxtubeRetractCommand()
        );
    }
}
