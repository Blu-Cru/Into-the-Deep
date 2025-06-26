package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;

public class SampleIntakeLeftPath extends PIDPathBuilder {
    public SampleIntakeLeftPath() {
        super();
        this.setPower(0.7)
                .addMappedPoint(-61, -45, 100, 4)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new WaitCommand(800),
                            new PivotRetractCommand(),
                            new WaitCommand(800),
                            new PreIntakeCommand(),
                            new WaitCommand(300),
                            new ExtensionCommand(8),
                            new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL),
                            new ClawOpenCommand(),
                            new TurretAngleCommand(1.0),
                            new WaitCommand(800),
                            new GrabCommand(),
                            new WaitCommand(300)
                    ).schedule();
                })
                .waitMillis(4000);
    }
}
