package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class SpinWristAngleCommand extends InstantCommand {
    public SpinWristAngleCommand(double angle) {
        super(() -> {
            Robot.getInstance().spinWrist.setAngle(angle);
        });

        addRequirements(Robot.getInstance().spinWrist);
    }
}
