package org.firstinspires.ftc.teamcode.blucru.common.commandbase.pusher;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class PushCommand extends SequentialCommandGroup {
    public PushCommand() {
        super(
                new PusherExtendCommand(),
                new WaitCommand(550),
                new PusherRetractCommand()
        );
    }
}
