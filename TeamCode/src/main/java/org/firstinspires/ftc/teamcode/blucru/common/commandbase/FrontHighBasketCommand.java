package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;

public class FrontHighBasketCommand extends SequentialCommandGroup {
    public FrontHighBasketCommand() {
        super(
                new ArmGlobalAngleCommand(1.4),
                new BoxtubeExtendCommand(Math.PI/2, 24),
                new WristUprightForwardCommand(),
                new WaitCommand(200),
                new ArmGlobalAngleCommand(0)
        );
    }
}
