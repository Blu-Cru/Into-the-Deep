package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class DepositSubSamplePath extends PIDPathBuilder {

    public DepositSubSamplePath(){
        super();
        this.setPower(1.0)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new PreIntakeCommand(),
                            new BoxtubeCommand(0, 0)
                    ).schedule();
                })
                .addMappedPoint(16, -50, 90, 10)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new TurretMotionProfileCommand(-1.0),
                            new WaitCommand(400),
                            new BoxtubeCommand(0, 10),
                            new PreIntakeCommand()
                    ).schedule();
                })
                .addMappedPoint(20, -60, 10, 7)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new WaitCommand(250),
                            new ClawOpenCommand()
                    ).schedule();
                })
                .waitMillis(400);
    }
}
