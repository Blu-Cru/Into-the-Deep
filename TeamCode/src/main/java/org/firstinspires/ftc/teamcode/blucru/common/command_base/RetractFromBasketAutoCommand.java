package org.firstinspires.ftc.teamcode.blucru.common.command_base;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;

public class RetractFromBasketAutoCommand extends SequentialCommandGroup {
    public RetractFromBasketAutoCommand() {
        super(
                new ArmGlobalAngleCommand(1.2),
                new WaitCommand(300),
                new BoxtubeRetractCommand(),
                new EndEffectorRetractCommand()
        );
    }
}
