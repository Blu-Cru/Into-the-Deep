package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class SpecimenIntakeBackFlatSpitCommand extends SequentialCommandGroup {
    public SpecimenIntakeBackFlatSpitCommand () {
        super(
                new ClawGrabCommand(),
                new ArmCommand(2.7),
                new BoxtubeCommand(1.2, 0),
                new SpinWristAngleCommand(0.8),
                new UpDownWristAngleCommand(-0.3),
                new WaitCommand(200),
                new TurretMotionProfileCommand(0.8),
                new WaitCommand(170),
                new ClawOpenCommand(),
                new ArmCommand(2.4),
                new WaitCommand(150),
                new SpecimenIntakeBackFlatCommand()
        );
    }
}
