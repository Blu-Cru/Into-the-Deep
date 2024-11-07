package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmRetractCommand extends InstantCommand {
    public ArmRetractCommand() {
        super(
                () -> Robot.getInstance().arm.retract()
        );
    }
}
