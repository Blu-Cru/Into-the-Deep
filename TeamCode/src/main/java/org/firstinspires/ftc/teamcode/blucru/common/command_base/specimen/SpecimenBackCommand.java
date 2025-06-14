package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class SpecimenBackCommand extends SequentialCommandGroup {
    public SpecimenBackCommand() {
        super(
//                new WristHorizontalCommand(),
                new ArmGlobalAngleCommand(Math.PI),
                new BoxtubeCommand(1.45, 10)
        );
    }
}
