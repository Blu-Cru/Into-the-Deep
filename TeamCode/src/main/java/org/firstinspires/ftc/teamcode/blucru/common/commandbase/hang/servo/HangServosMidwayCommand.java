package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.servo;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class HangServosMidwayCommand extends InstantCommand {
    public HangServosMidwayCommand() {
        super(
                () -> Robot.getInstance().hangServos.midway()
        );
    }
}
