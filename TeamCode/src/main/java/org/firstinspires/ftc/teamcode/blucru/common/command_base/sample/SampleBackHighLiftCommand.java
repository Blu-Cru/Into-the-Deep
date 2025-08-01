package org.firstinspires.ftc.teamcode.blucru.common.command_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class SampleBackHighLiftCommand extends SequentialCommandGroup {
    public SampleBackHighLiftCommand() {
        super(
//                new WristUprightBackwardCommand(),
                new BoxtubeCommand(Math.PI/2, 25.9),
                new ArmGlobalAngleCommand(1.6)
        );
    }
}
