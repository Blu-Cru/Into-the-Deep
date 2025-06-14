package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmGlobalAngleCommand extends InstantCommand {
    public ArmGlobalAngleCommand (double globalAngle) {
        super(
                () -> Robot.getInstance().arm.setGlobalAngle(globalAngle)
        );

        addRequirements(Robot.getInstance().arm);
    }
}
