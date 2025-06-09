package org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristAngleCommand;

public class PreIntakeCommand extends SequentialCommandGroup {
    public PreIntakeCommand() {
        super(
                new ArmMotionProfileCommand(-0.1),
                new UpDownWristAngleCommand(-Math.PI/2 + 0.1),
                new WaitCommand(170),
                new ClawOpenCommand()
        );
    }
}
