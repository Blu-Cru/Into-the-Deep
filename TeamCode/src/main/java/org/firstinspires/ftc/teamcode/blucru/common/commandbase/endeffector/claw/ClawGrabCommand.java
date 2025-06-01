package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ClawGrabCommand extends InstantCommand {
    public ClawGrabCommand() {
        super(
                () -> Robot.getInstance().claw.grab()
        );

        addRequirements(Robot.getInstance().claw);
    }
}
