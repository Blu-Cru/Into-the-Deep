package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.slides;

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
