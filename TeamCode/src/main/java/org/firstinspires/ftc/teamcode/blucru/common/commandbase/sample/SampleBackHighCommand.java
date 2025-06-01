package org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;

public class SampleBackHighCommand extends SequentialCommandGroup {
    public SampleBackHighCommand() {
        super(
//                new WristUprightBackwardCommand(),
                new BoxtubeCommand(Math.PI/2, 25.9),
                new ArmGlobalAngleCommand(2.6)
        );
    }
}
