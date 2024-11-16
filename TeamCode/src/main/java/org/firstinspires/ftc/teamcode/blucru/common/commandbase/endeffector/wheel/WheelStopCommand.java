package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WheelStopCommand extends InstantCommand {
    public WheelStopCommand() {
        super(
                () -> Robot.getInstance().wheel.stop()
        );

        addRequirements(Robot.getInstance().wheel);
    }
}
