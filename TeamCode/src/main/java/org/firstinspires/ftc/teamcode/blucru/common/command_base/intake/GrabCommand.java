package org.firstinspires.ftc.teamcode.blucru.common.command_base.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class GrabCommand extends SequentialCommandGroup {
    public GrabCommand() {
        super(
                new ArmCommand(-0.45),
                new UpDownWristAngleCommand(-Math.PI/2 + 0.45),
                new WaitCommand(200),
                new ClawGrabCommand(),
                new WaitCommand(120),
                new ArmCommand(0),
                new UpDownWristAngleCommand(0),
                new WaitCommand(100),
                new SpinWristCenterCommand(),
                new ClawLooseCommand()
        );
    }
}
