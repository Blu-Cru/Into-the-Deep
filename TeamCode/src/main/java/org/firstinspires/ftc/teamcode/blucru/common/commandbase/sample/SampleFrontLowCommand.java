package org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;

public class SampleFrontLowCommand extends SequentialCommandGroup {
    public SampleFrontLowCommand() {
        super(
                new BoxtubeCommand(1.0, 16.6),
                new WristUprightForwardCommand(),
                new WaitCommand(100),
                new ArmGlobalAngleCommand(-0.2)
        );
    }
}
