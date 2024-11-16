package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmPreIntakeCommand extends InstantCommand {
    public ArmPreIntakeCommand() {
        super(
                () -> Robot.getInstance().arm.preIntake()
        );

        addRequirements(Robot.getInstance().arm);
    }
}
