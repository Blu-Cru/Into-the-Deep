package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PivotCommand extends InstantCommand {
    public PivotCommand(double angleRad) {
        super(() -> {
            Robot.getInstance().pivot.pidTo(angleRad);
        });

        addRequirements(Robot.getInstance().pivot);
    }
}
