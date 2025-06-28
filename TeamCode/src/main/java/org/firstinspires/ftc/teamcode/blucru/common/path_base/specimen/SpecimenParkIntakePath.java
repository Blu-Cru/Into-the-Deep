package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;

public class SpecimenParkIntakePath extends PIDPathBuilder {
    public SpecimenParkIntakePath() {
        super();
        this.setPower(0.8)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(150),
                            new FullRetractCommand()
                    ).schedule();
                })
                .addMappedPoint(12, -50, 90, 5)
                .addMappedPoint(16, -60, 0, 10)
                .callback(() ->
                        new SequentialCommandGroup(
                                new ClawOpenCommand(),
                                new PreIntakeCommand(),
                                new WaitCommand(300),
                                new ExtensionCommand(13),
                                new TurretMotionProfileCommand(-0.6),
                                new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL),
                                new WaitCommand(600),
                                new GrabCommand()
                        ).schedule()
                )
                .waitMillis(1200);
    }
}
