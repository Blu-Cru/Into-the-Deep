package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmDisableCommand extends InstantCommand {
    public ArmDisableCommand() {
        super(
                () -> {
                    Robot.getInstance().arm.disable();
                }
        );

        addRequirements(Robot.getInstance().arm);
    }
}
