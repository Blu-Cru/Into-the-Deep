package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.RetractFromBasketCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.Claw;

public class CrossDepositYellowSamplePath extends PIDPathBuilder {
    public CrossDepositYellowSamplePath() {
        super();
        this.setPower(1.0)
                .callback(() -> new FullRetractCommand().schedule())
                .addMappedPoint(-30, -53, 0, 20)
                .setPower(0.4)
                .callback(() -> {
                    new SampleBackHighCommand().schedule();
                })
                .addMappedPoint(-50, -58, 0)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(170),
                            new RetractFromBasketCommand()
                    ).schedule();
                })
                .waitMillis(600);
    }
}
