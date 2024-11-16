package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightBackwardCommand;

public class BasketBackHighCommand extends SequentialCommandGroup {
    public BasketBackHighCommand() {
        super(
                new BoxtubeExtendCommand(Math.PI/2, 24),
                new WristUprightBackwardCommand(),
                new ArmGlobalAngleCommand(2.6)
        );
    }
}
