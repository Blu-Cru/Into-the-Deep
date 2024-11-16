package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WheelReverseCommand extends InstantCommand {
    public WheelReverseCommand() {
        super(
                () -> Robot.getInstance().wheel.reverse()
        );

        addRequirements(Robot.getInstance().wheel);
    }
}
