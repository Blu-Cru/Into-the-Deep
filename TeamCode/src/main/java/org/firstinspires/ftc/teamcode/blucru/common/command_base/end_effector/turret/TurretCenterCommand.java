package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurretCenterCommand extends InstantCommand {
    public TurretCenterCommand () {
        super(() -> Robot.getInstance().turret.setAngle(0));

        addRequirements(Robot.getInstance().turret);
    }
}
