package org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightBackwardCommand;

public class SampleBackHighCommand extends SequentialCommandGroup {
    public SampleBackHighCommand() {
        super(
                new BoxtubeExtendCommand(Math.PI/2, 24),
                new WristUprightBackwardCommand(),
                new ArmGlobalAngleCommand(2.6)
        );
    }
}
