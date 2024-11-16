package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristHorizontalCommand;

public class BackHighSpecimenCommand extends SequentialCommandGroup {
    public BackHighSpecimenCommand() {
        super(
                new BoxtubeExtendCommand(1.4, 6),
                new WristHorizontalCommand(),
                new ArmGlobalAngleCommand(2.5)
        );
    }
}
