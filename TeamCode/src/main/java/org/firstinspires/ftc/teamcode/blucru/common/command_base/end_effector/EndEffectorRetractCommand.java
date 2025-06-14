package org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristRetractCommand;

public class EndEffectorRetractCommand extends SequentialCommandGroup {
    public EndEffectorRetractCommand() {
        super(
                new ClawLooseCommand(),
                new ArmRetractCommand(),
                new SpinWristCenterCommand(),
                new TurretCenterCommand(),
                new UpDownWristRetractCommand()
        );
    }
}
