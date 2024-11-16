package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WristOppositeCommand extends InstantCommand {
    public WristOppositeCommand() {
        super(
                () -> Robot.getInstance().wrist.opposite()
        );

        addRequirements(Robot.getInstance().wrist);
    }
}

