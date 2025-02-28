package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.motor;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class HangMotorHighBarCommand extends InstantCommand {
    public HangMotorHighBarCommand() {
        super(
                () -> Robot.getInstance().hangMotor.pidHighBar()
        );
    }
}
