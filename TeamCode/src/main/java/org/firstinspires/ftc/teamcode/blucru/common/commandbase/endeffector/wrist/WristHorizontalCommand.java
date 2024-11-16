package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WristHorizontalCommand extends InstantCommand {
    public WristHorizontalCommand() {
        super(
                () -> Robot.getInstance().wrist.horizontal()
        );

        addRequirements(Robot.getInstance().wrist);
    }
}
