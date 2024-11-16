package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ExtensionRetractCommand extends InstantCommand {
    public ExtensionRetractCommand() {
        super(
                () -> {
                     Robot.getInstance().extension.retract();
                }
        );

        addRequirements(Robot.getInstance().extension);
    }
}
