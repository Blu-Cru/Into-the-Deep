package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmBottomHardStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;

public class HooksOnHighBarCommand extends SequentialCommandGroup {
    public HooksOnHighBarCommand() {
        super(
                new PivotCommand(1.3),
                new ExtensionCommand(11.5),
                new ArmBottomHardStopCommand()
        );
    }
}
