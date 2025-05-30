package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;

public class BoxtubeRetractFromTopBarCommand extends SequentialCommandGroup {
    public BoxtubeRetractFromTopBarCommand() {
        super(
                new ExtensionRetractCommand(),
                new ArmRetractCommand(),
                new WaitCommand(600),
                new PivotCommand(0.5)
        );
    }
}
