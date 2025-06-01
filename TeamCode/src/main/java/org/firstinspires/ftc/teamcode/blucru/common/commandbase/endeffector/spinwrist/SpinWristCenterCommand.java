package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class SpinWristCenterCommand extends InstantCommand {
    public SpinWristCenterCommand() {
        super(() -> {
            Robot.getInstance().spinWrist.center();
        });

        addRequirements(Robot.getInstance().spinWrist);
    }
}
