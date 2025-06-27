package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.RetractFromBasketCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;

public class SampleHighDepositPreloadPath extends PIDPathBuilder {
    public SampleHighDepositPreloadPath(boolean retractAfterDone) {
        super();
        this.setPower(0.25)
                .addMappedPoint(-53, -53, 45)
                .callback(() -> {
                    if(retractAfterDone) {
                        new SequentialCommandGroup(
                                new ArmCommand(0.4),
                                new WaitCommand(80),
                                new ClawOpenCommand(),
                                new WaitCommand(120),
                                new RetractFromBasketCommand()
                        ).schedule();
                    } else {
                        new SequentialCommandGroup(
                                new ArmCommand(0.4),
                                new WaitCommand(80),
                                new ClawOpenCommand()
                        ).schedule();
                    }
                })
                .waitMillis(500);
    }

    public SampleHighDepositPreloadPath() {
        this(true);
    }
}
