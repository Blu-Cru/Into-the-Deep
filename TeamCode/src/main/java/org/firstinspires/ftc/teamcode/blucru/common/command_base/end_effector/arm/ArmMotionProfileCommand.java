package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ArmMotionProfileCommand extends InstantCommand {
    public ArmMotionProfileCommand(double angle) {
        super(
                () -> {
                    Robot.getInstance().arm.setMotionProfileAngle(angle);
                }
        );

        addRequirements(Robot.getInstance().arm);
    }
}
