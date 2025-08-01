package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.slides;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class HangServosHangComamnd extends InstantCommand {
    public HangServosHangComamnd() {
        super(
                () -> {
                    Robot.getInstance().slideHangServos.retract();
                }
        );

        addRequirements(Robot.getInstance().slideHangServos);
    }
}
