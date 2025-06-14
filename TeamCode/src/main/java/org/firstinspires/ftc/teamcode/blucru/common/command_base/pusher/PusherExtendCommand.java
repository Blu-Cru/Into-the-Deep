package org.firstinspires.ftc.teamcode.blucru.common.command_base.pusher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PusherExtendCommand extends InstantCommand {
    public PusherExtendCommand() {
        super(() -> {
             Robot.getInstance().pusher.extend();
        });

        addRequirements(Robot.getInstance().pusher);
    }
}
