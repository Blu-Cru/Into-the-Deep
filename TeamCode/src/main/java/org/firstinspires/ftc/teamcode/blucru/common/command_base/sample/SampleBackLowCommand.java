package org.firstinspires.ftc.teamcode.blucru.common.command_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class SampleBackLowCommand extends SequentialCommandGroup {
    public SampleBackLowCommand() {
        super(
                new BoxtubeCommand(Math.PI/2, 13.5),
//                new WristUprightBackwardCommand(),
                new ArmGlobalAngleCommand(3.2)
        );
    }
}
