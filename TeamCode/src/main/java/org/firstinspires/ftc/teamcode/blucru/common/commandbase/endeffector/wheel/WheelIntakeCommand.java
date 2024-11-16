package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class WheelIntakeCommand extends InstantCommand {
    public WheelIntakeCommand() {
        super(
                () -> Robot.getInstance().wheel.intake()
        );

        addRequirements(Robot.getInstance().wheel);
    }
}
