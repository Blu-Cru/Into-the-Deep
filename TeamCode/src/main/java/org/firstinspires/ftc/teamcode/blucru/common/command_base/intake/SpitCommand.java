package org.firstinspires.ftc.teamcode.blucru.common.command_base.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristRetractCommand;

public class SpitCommand extends SequentialCommandGroup {
    public SpitCommand() {
        super(
                new UpDownWristAngleCommand(-2.5),
                new WaitCommand(200),
                new ClawOpenCommand(),
                new WaitCommand(150),
                new ClawLooseCommand(),
                new UpDownWristRetractCommand()
        );
    }
}
