package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ClawOpenCommand extends InstantCommand {
    public ClawOpenCommand() {
        super(
                () -> Robot.getInstance().claw.release()
        );

        addRequirements(Robot.getInstance().claw);
    }
}
