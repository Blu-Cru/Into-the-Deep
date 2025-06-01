package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ClawLooseCommand extends InstantCommand {
    public ClawLooseCommand() {
        super(() -> {
            Robot.getInstance().claw.looseGrab();
        });

        addRequirements(Robot.getInstance().claw);
    }
}
