package org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;

public class SampleBackLowCommand extends SequentialCommandGroup {
    public SampleBackLowCommand() {
        super(
                new BoxtubeCommand(Math.PI/2, 13.5),
//                new WristUprightBackwardCommand(),
                new ArmGlobalAngleCommand(3.2)
        );
    }
}
