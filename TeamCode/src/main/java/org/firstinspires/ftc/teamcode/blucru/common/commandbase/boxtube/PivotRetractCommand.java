package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PivotRetractCommand extends InstantCommand {
    public PivotRetractCommand() {
        super(
                () -> {
                    Robot.getInstance().pivot.retract();
                }
        );

        addRequirements(Robot.getInstance().pivot);
    }
}
