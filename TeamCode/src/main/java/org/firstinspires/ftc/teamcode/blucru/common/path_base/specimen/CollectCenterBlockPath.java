package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;

public class CollectCenterBlockPath extends PIDPathBuilder {
    public CollectCenterBlockPath() {
        super();
        this.setPower(0.85)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new PreIntakeCommand(),
                            new WaitCommand(100),
                            new TurretMotionProfileCommand(1.4)
                    ).schedule();
                })
                .addMappedPoint(39, -40, 14, 4)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new PreIntakeCommand(),
                            new WaitCommand(150),
                            new ExtensionCommand(12),
                            new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL)
                    ).schedule();
                })
                .waitMillis(1300);
    }
}
