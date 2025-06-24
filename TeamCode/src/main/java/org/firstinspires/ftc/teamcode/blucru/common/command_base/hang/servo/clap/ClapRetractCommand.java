package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.clap;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ClapRetractCommand extends InstantCommand {
    public ClapRetractCommand() {
        super(() -> {
            Robot.getInstance().clapServos.retract();
        });
    }
}
