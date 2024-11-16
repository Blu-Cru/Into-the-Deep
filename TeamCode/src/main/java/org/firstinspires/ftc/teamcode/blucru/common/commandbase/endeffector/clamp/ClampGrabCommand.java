package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ClampGrabCommand extends InstantCommand {
    public ClampGrabCommand() {
        super(
                () -> Robot.getInstance().clamp.grab()
        );

        addRequirements(Robot.getInstance().clamp);
    }
}
