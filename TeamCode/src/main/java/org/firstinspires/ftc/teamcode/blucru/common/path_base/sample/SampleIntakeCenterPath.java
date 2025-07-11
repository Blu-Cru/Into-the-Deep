package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;

public class SampleIntakeCenterPath extends PIDPathBuilder {
    public SampleIntakeCenterPath() {
        super();
        this.setPower(0.7)
                .addMappedPoint(-53.2, -50, 90, 4)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new BoxtubeRetractCommand(),
                            new ArmCommand(Math.PI/2),
                            new WaitCommand(150),
                            new PreIntakeCommand(),
                            new WaitCommand(200),
                            new ExtensionCommand(10),
                            new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL),
                            new ClawOpenCommand(),
                            new TurretAngleCommand(1.1),
                            new WaitCommand(300),
                            new GrabCommand()
                    ).schedule();
                })
                .waitMillis(1600);
    }
}
