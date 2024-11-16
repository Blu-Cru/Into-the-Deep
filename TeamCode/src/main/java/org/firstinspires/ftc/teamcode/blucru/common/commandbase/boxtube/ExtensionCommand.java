package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ExtensionCommand extends InstantCommand {
    public ExtensionCommand(double inches) {
        super(
                () -> {
                     Robot.getInstance().extension.pidTo(inches);
                }
        );

        addRequirements(Robot.getInstance().extension);
    }
}
