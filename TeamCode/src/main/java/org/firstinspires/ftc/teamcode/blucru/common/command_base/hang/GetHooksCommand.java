package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmBackHardStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.clap.ClapCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.clap.ClapRetractCommand;

public class GetHooksCommand extends SequentialCommandGroup {
    public GetHooksCommand () {
        super(
                new ArmBackHardStopCommand(),
                new UpDownWristAngleCommand(-1.2),
                new SpinWristAngleCommand(Math.PI),
                new ExtensionCommand(0.0),
                new ClawOpenCommand(),
                new ClapCenterCommand(),
                new WaitCommand(350),
                new ArmCommand(2.35),
                new WaitCommand(350),
                new ClawGrabCommand(),
                new WaitCommand(350),
                new ExtensionCommand(6),
                new WaitCommand(350),
                new ArmCommand(1.9),
                new ClapRetractCommand(),
                new WaitCommand(300),
                new BoxtubeCommand(Math.PI/2, 0)
        );
    }
}
