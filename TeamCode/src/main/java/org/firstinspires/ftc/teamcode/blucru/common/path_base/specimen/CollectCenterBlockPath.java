package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;

public class CollectCenterBlockPath extends PIDPathBuilder {
    public CollectCenterBlockPath() {
        super();
        this.setPower(0.9)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new PreIntakeCommand(),
                            new TurretMotionProfileCommand(1.2),
                            new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL)
                    ).schedule();
                })
                .addMappedPoint(39, -39, 17, 13)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new WaitCommand(150),
                            new ExtensionCommand(9.5)
                    ).schedule();
                })
                .addMappedPoint(39, -39, 17)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new GrabCommand(),
                            new WaitCommand(360),
                            new TurretMotionProfileCommand(-1.0)
                    ).schedule();
                })
                .waitMillis(400);
    }
}
