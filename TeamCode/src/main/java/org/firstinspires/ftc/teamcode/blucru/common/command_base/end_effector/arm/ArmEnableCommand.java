package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmEnableCommand extends InstantCommand {
    public ArmEnableCommand() {
        super(
                () -> {
                    Robot.getInstance().arm.enable();
                }
        );

        addRequirements(Robot.getInstance().arm);
    }
}
