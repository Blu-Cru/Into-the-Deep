package org.firstinspires.ftc.teamcode.blucru.common.command_base.pusher;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PusherRetractCommand extends InstantCommand {
    public PusherRetractCommand() {
        super(() -> {
             Robot.getInstance().pusher.retract();
        });

        addRequirements(Robot.getInstance().pusher);
    }
}
