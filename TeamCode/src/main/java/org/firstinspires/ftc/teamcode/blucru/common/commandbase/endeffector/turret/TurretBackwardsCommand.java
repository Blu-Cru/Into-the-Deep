package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret;

import com.arcrobotics.ftclib.command.InstantCommand;

public class TurretBackwardsCommand extends TurretMotionProfileCommand {
    public TurretBackwardsCommand() {
        super(Math.PI);
    }
}
