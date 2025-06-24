package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.pto;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PTOEngageCommand extends InstantCommand {
    public PTOEngageCommand() {
        super(
                () -> Robot.getInstance().ptoServos.engage()
        );

        addRequirements(Robot.getInstance().ptoServos);
    }
}
