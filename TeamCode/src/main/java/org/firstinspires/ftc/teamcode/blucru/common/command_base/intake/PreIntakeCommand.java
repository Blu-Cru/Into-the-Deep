package org.firstinspires.ftc.teamcode.blucru.common.command_base.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class PreIntakeCommand extends SequentialCommandGroup {
    public PreIntakeCommand() {
        super(
                new ArmMotionProfileCommand(-0.05),
                new UpDownWristAngleCommand(-Math.PI/2 + 0.05),
                new WaitCommand(170),
                new ClawOpenCommand()
        );
    }
}
