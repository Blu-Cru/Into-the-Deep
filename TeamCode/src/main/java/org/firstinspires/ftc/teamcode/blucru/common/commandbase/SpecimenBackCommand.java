package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristOppositeCommand;

public class SpecimenBackCommand extends SequentialCommandGroup {
    public SpecimenBackCommand() {
        super(
                new WristOppositeCommand(),
                new ArmGlobalAngleCommand(2.7),
                new BoxtubeExtendCommand(1.45, 8.5)
        );
    }
}
