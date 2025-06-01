package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist;

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
