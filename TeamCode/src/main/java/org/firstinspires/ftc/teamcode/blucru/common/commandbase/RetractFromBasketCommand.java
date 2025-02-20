package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;

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
