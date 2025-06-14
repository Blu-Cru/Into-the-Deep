package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class HangServosMidwayCommand extends InstantCommand {
    public HangServosMidwayCommand() {
        super(
                () -> Robot.getInstance().slideHangServos.midway()
        );
    }
}
