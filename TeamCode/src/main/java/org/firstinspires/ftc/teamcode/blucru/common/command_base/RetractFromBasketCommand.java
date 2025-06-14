package org.firstinspires.ftc.teamcode.blucru.common.command_base;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class RetractFromBasketCommand extends SequentialCommandGroup {
    public RetractFromBasketCommand() {
        super(
                new ArmGlobalAngleCommand(1.5),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new EndEffectorRetractCommand()
                        ),
//                        new SequentialCommandGroup(
                                new BoxtubeRetractCommand()
//                                new EndEffectorRetractCommand()
//                        )
                )
        );
    }
}
