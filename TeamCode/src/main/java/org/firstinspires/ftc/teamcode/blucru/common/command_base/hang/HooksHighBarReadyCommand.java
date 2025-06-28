package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class HooksHighBarReadyCommand extends SequentialCommandGroup {
    public HooksHighBarReadyCommand() {
        super(
                // pivot 1.3
                // extension 10.8
                new PivotCommand(1.3),
                new WaitCommand(300),
                new ExtensionCommand(11.5),
                new ArmCommand(0.3),
                new UpDownWristAngleCommand(-Math.PI/2)
        );
    }
}
