package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightBackwardCommand;

public class BackLowBasketCommand extends SequentialCommandGroup {
    public BackLowBasketCommand() {
        super(
                new BoxtubeExtendCommand(Math.PI/2, 13.5),
                new WristUprightBackwardCommand(),
                new ArmGlobalAngleCommand(3.4)
        );
    }
}
