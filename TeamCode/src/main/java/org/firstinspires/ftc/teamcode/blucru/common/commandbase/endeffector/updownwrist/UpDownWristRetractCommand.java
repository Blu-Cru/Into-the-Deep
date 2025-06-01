package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class UpDownWristRetractCommand extends InstantCommand {
    public UpDownWristRetractCommand() {
        super(() -> {
            Robot.getInstance().upDownWrist.retract();
        });

        addRequirements(Robot.getInstance().upDownWrist);
    }
}
