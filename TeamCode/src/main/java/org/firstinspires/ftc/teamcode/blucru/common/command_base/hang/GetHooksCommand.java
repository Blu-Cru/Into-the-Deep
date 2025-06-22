package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmBackHardStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.clap.ClapCenterCommand;

public class GetHooksCommand extends SequentialCommandGroup {
    public GetHooksCommand () {
        super(
                new ArmBackHardStopCommand(),
                new UpDownWristAngleCommand(-2.5 + Math.PI/2),
                new ClapCenterCommand(),
                new WaitCommand(700),
                new ArmCommand(2.5),
                new WaitCommand(500),
                new ClawGrabCommand()
        );
    }
}
