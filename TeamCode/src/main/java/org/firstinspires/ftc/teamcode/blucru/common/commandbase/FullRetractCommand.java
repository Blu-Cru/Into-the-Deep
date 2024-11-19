package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;

public class FullRetractCommand extends SequentialCommandGroup {
    public FullRetractCommand() {
        super(
                new EndEffectorRetractCommand(),
                new BoxtubeRetractCommand()
        );
    }
}
