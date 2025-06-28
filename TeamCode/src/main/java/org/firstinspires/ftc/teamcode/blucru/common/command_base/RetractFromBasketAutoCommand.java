package org.firstinspires.ftc.teamcode.blucru.common.command_base;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class RetractFromBasketAutoCommand extends SequentialCommandGroup {
    public RetractFromBasketAutoCommand() {
        super(
                new ArmGlobalAngleCommand(1.2),
                new UpDownWristAngleCommand(-0.3),
                new WaitCommand(300),
                new BoxtubeRetractCommand(),
                new EndEffectorRetractCommand()
        );
    }
}
