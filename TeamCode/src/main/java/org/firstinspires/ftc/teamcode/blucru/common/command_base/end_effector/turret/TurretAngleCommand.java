package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurretAngleCommand extends InstantCommand {
    public TurretAngleCommand(double angle) {
        super(() -> {
            Robot.getInstance().turret.setAngle(angle);
        });

        addRequirements(Robot.getInstance().turret);
    }
}
