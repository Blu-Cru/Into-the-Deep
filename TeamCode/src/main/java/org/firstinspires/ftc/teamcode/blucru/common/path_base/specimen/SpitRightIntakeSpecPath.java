package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpitRightIntakeSpecPath extends PIDPathBuilder {
    public SpitRightIntakeSpecPath() {
        super();
        this.setPower(0.9)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new EndEffectorRetractCommand(),
                            new BoxtubeCommand(Math.PI/2, 4),
                            new WaitCommand(500),
                            new TurretMotionProfileCommand(-0.5)
                    ).schedule();
                })
                .addMappedPoint(39, -54, 90, 5)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(150),
                            new SpecimenIntakeBackClipCommand()
                    ).schedule();
                })
                .setPower(0.25)
                .addMappedPoint(36, -62, 90, 2)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new WaitCommand(140),
                            new ExtensionCommand(7.0)
                    ).schedule();
                })
                .waitMillis(250);
    }
}
