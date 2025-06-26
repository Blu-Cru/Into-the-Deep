package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
                new ExtensionCommand(0.5),
                new ClawOpenCommand(),
                new ClapCenterCommand(),
                new WaitCommand(350),
                new ArmCommand(2.4),
                new WaitCommand(500),
                new ClawGrabCommand(),
                new WaitCommand(400),
                new ArmCommand(2.0),
                new ExtensionCommand(4),
                new WaitCommand(700),
                new ClapRetractCommand()
        );
    }
}
