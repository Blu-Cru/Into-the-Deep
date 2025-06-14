package org.firstinspires.ftc.teamcode.blucru.common.command_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class SampleFrontLowCommand extends SequentialCommandGroup {
    public SampleFrontLowCommand() {
        super(
                new BoxtubeCommand(1.2, 16.6),
//                new WristUprightForwardCommand(),
                new WaitCommand(100),
                new ArmGlobalAngleCommand(-0.2)
        );
    }
}
