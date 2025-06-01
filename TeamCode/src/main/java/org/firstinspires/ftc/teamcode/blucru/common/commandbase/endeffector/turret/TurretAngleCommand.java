package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret;

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
