package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class UpDownWristAngleCommand extends InstantCommand {
    public UpDownWristAngleCommand (double angle) {
        super(() -> {
            Robot.getInstance().upDownWrist.setAngle(angle);
        });

        addRequirements(Robot.getInstance().upDownWrist);
    }
}
