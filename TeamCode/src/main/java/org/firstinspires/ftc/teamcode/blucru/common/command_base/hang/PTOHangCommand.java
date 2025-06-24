package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PTOHangCommand extends InstantCommand {
    public PTOHangCommand () {
        super(
                () -> Robot.getInstance().ptoDt.setMotionProfileInches(-5.0)
        );
        addRequirements(Robot.getInstance().ptoDt);
    }
}
