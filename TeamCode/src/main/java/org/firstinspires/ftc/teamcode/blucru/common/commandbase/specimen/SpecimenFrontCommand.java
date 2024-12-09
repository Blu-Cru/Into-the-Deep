package org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristOppositeCommand;

public class SpecimenFrontCommand extends SequentialCommandGroup {
    public SpecimenFrontCommand() {
        super(
                new WristOppositeCommand(),
                new BoxtubeExtendCommand(1.35, 8.5),
                new ArmGlobalAngleCommand(-0.3)
        );
    }
}