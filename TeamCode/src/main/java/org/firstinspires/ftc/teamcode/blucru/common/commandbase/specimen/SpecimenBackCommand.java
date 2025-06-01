package org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;

public class SpecimenBackCommand extends SequentialCommandGroup {
    public SpecimenBackCommand() {
        super(
//                new WristHorizontalCommand(),
                new ArmGlobalAngleCommand(Math.PI),
                new BoxtubeCommand(1.45, 10)
        );
    }
}
