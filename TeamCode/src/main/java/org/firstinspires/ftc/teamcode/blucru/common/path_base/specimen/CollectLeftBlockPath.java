package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;

public class CollectLeftBlockPath extends PIDPathBuilder {
    public CollectLeftBlockPath() {
        super();
        this.setPower(0.8)
                .addMappedPoint(29, -44.5, 30,8)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new PivotRetractCommand(),
                            new PreIntakeCommand(),
                            new WaitCommand(150),
                            new ExtensionCommand(10),
                            new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL),
                            new TurretMotionProfileCommand(0.3)
                    ).schedule();
                })
                .waitMillis(3000);
    }
}
