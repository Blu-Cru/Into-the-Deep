package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristOppositeCommand;

public class SpecimenFrontCommand extends SequentialCommandGroup {
    public SpecimenFrontCommand() {
        super(
                new WristOppositeCommand(),
                new ArmGlobalAngleCommand(0.45),
                new BoxtubeExtendCommand(1.4, 8.5)
        );
    }
}
