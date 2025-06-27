package org.firstinspires.ftc.teamcode.blucru.common.command_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class SampleBackHighAutoCommand extends SequentialCommandGroup {
    public SampleBackHighAutoCommand () {
        super(
                new ArmCommand(0.0),
                new BoxtubeCommand(Math.PI/2, 23.5),
                new UpDownWristAngleCommand(1.3),
                new SpinWristAngleCommand(Math.PI),
                new TurretCenterCommand()
        );
    }
}
