package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.servo;

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
