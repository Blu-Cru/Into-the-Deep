package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;

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
