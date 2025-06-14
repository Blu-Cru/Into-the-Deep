package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist;

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
