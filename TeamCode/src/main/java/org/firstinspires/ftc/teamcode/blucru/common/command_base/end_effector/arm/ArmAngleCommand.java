package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmAngleCommand extends InstantCommand {
    public ArmAngleCommand(double angle) {
        super(() -> Robot.getInstance().arm.setAngle(angle));

        addRequirements(Robot.getInstance().arm);
    }
}
