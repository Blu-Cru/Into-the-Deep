package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpitPath extends PIDPathBuilder {
    public SpitPath() {
        super();
        this.setPower(0.9)
                .schedule(new SequentialCommandGroup(
                        new ExtensionCommand(4),
                        new PreIntakeCommand()
                ))
                .addMappedPoint(38, -50, -30, 7)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ExtensionCommand(6),
                            new TurretMotionProfileCommand(-1.0),
                            new WaitCommand(100),
                            new ClawOpenCommand(),
                            new ExtensionCommand(3),
                            new WaitCommand(100),
                            new ArmCommand(1.0),
                            new TurretCenterCommand()
                    ).schedule();
                })
                .waitMillis(150);
    }
}
