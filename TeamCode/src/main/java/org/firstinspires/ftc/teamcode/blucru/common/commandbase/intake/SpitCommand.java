package org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristRetractCommand;

public class SpitCommand extends SequentialCommandGroup {
    public SpitCommand() {
        super(
                new UpDownWristAngleCommand(-2.5),
                new WaitCommand(250),
                new ClawOpenCommand(),
                new WaitCommand(300),
                new ClawLooseCommand(),
                new UpDownWristRetractCommand()
        );
    }
}
