package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.pto;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PTODisengageCommand extends InstantCommand {
    public PTODisengageCommand() {
        super(
                () -> Robot.getInstance().ptoServos.disengage()
        );

        addRequirements(Robot.getInstance().ptoServos);
    }
}
