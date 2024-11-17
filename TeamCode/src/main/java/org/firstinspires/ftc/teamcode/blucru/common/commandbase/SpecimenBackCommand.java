package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristHorizontalCommand;

public class SpecimenBackCommand extends SequentialCommandGroup {
    public SpecimenBackCommand() {
        super(
                new WristHorizontalCommand(),
                new ArmGlobalAngleCommand(Math.PI),
                new BoxtubeExtendCommand(1.45, 6.5)
        );
    }
}
