package org.firstinspires.ftc.teamcode.blucru.common.command_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class SampleFrontHighCommand extends SequentialCommandGroup {
    public SampleFrontHighCommand() {
        super(
                new ArmGlobalAngleCommand(1.4),
                new BoxtubeCommand(Math.PI/2, 26),
//                new WristUprightForwardCommand(),
                new WaitCommand(200),
                new ArmGlobalAngleCommand(0.5)
        );
    }
}
