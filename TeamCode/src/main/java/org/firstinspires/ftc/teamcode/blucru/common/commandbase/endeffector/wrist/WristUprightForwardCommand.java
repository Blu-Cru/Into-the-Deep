package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WristUprightForwardCommand extends InstantCommand {
    public WristUprightForwardCommand() {
        super(
                () -> Robot.getInstance().wrist.front()
        );

        addRequirements(Robot.getInstance().wrist);
    }
}
