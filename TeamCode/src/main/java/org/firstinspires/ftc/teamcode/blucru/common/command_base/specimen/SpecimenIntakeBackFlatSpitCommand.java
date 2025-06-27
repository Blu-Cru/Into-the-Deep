package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristRetractCommand;

public class SpecimenIntakeBackFlatSpitCommand extends SequentialCommandGroup {
    public SpecimenIntakeBackFlatSpitCommand () {
        super(
                new ClawGrabCommand(),
                new SpinWristCenterCommand(),
                new TurretCenterCommand(),
                new UpDownWristRetractCommand(),
                new ArmCommand(2.7),
                new BoxtubeCommand(1.3, 0),
                new SpinWristAngleCommand(Math.PI),
                new UpDownWristAngleCommand(-1.3),
                new WaitCommand(200),
                new TurretMotionProfileCommand(0.45),
                new WaitCommand(170),
                new SpecimenIntakeBackFlatCommand()
        );
    }
}
