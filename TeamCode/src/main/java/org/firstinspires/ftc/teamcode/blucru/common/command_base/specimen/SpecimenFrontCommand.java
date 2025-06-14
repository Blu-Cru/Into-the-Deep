package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class SpecimenFrontCommand extends SequentialCommandGroup {
    public SpecimenFrontCommand() {
        super(
//                new WristOppositeCommand(),
                new BoxtubeCommand(1.35, 8.5),
                new ArmGlobalAngleCommand(-0.3)
        );
    }
}