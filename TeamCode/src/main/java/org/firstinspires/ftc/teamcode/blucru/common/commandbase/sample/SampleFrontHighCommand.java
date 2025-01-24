package org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;

public class SampleFrontHighCommand extends SequentialCommandGroup {
    public SampleFrontHighCommand() {
        super(
                new ArmGlobalAngleCommand(1.4),
                new BoxtubeCommand(Math.PI/2, 26),
                new WristUprightForwardCommand(),
                new WaitCommand(200),
                new ArmGlobalAngleCommand(0.5)
        );
    }
}
