package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystem.endeffector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmDropToGroundCommand extends InstantCommand {
    public ArmDropToGroundCommand() {
        super(
                () -> Robot.getInstance().arm.dropToGround()
        );
    }
}