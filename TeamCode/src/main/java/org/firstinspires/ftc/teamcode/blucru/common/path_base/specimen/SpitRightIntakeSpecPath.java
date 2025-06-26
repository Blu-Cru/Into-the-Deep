package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpitRightIntakeSpecPath extends PIDPathBuilder {
    public SpitRightIntakeSpecPath() {
        super();
        this.setPower(0.9)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new EndEffectorRetractCommand(),
                            new BoxtubeCommand(1.0, 0),
                            new SpecimenIntakeBackFlatSpitCommand()
                    ).schedule();
                })
                .addMappedPoint(39, -54, 90, 2)
//                .callback(() -> {
//                    new SequentialCommandGroup(
//                            new UpDownWristAngleCommand(-1.0),
//                            new WaitCommand(150),
//                            new ClawOpenCommand()
//                    ).schedule();
//                })
                .waitMillis(150)
                .callback(() -> {
                    new SpecimenIntakeBackFlatCommand().schedule();
                })
                .setPower(0.25)
                .addMappedPoint(36, -61, 90, 2)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new WaitCommand(140),
                            new PivotCommand(1.0),
                            new UpDownWristAngleCommand(-2.0)
                    ).schedule();
                })
                .waitMillis(250);
    }
}
