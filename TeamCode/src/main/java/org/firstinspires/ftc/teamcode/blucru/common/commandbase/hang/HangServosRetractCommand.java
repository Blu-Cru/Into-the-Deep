package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class HangServosRetractCommand extends InstantCommand {
    public HangServosRetractCommand() {
        super(() -> Robot.getInstance().hangServos.retract());
        addRequirements(Robot.getInstance().hangServos);
    }
}
