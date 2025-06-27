package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;

public class CollectLeftBlockPath extends PIDPathBuilder {
    public CollectLeftBlockPath() {
        super();
        this.setPower(0.9)
                .addMappedPoint(29, -44.5, 40,18)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new PivotRetractCommand(),
                            new PreIntakeCommand(),
                            new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL),
                            new ExtensionCommand(11),
                            new WaitCommand(100),
                            new TurretMotionProfileCommand(0.8)
                    ).schedule();
                })
                .addMappedPoint(29, -44.5, 40)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new WaitCommand(100),
                            new GrabCommand(),
                            new WaitCommand(360),
                            new TurretMotionProfileCommand(-1.0)
                    ).schedule();
                })
                .waitMillis(500);
    }
}
