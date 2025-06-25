package org.firstinspires.ftc.teamcode.blucru.common.command_base.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class PreIntakeCommand extends SequentialCommandGroup {
    public PreIntakeCommand() {
        super(
                new ArmCommand(0.15),
                new UpDownWristAngleCommand(-Math.PI/2 + 0.05)
        );
    }
}
