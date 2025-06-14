package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.servo;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class HangServosReleaseCommand extends InstantCommand {
    public HangServosReleaseCommand() {
        super(
                () -> {
                    Robot.getInstance().slideHangServos.release();
                }
        );

        addRequirements(Robot.getInstance().slideHangServos);
    }
}
