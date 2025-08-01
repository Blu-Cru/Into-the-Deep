package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class SpecimenIntakeBackFlatCommand extends SequentialCommandGroup {
    public SpecimenIntakeBackFlatCommand() {
        super(
                new ClawOpenCommand(),
                new ArmCommand(2.8),
                new UpDownWristAngleCommand(-1.5),
                new SpinWristAngleCommand(Math.PI),
                new TurretCenterCommand(),
                new WaitCommand(100),
                new BoxtubeCommand(1.6, 0)
//                new WaitCommand(200),
//                new ClawOpenCommand()
        );
    }
}
