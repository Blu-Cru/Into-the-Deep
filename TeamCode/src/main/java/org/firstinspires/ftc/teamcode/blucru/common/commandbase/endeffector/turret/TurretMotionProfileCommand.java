package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurretMotionProfileCommand extends InstantCommand {
    public TurretMotionProfileCommand(double angle) {
        super(() -> Robot.getInstance().turret.setMotionProfileAngle(angle));

        addRequirements(Robot.getInstance().turret);
    }
}
