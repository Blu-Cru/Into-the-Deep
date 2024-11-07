package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.wrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WristUprightBackwardCommand extends InstantCommand {
    public WristUprightBackwardCommand() {
        super(
                () -> Robot.getInstance().wrist.uprightBackward()
        );
    }
}
