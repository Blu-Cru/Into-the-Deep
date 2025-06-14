package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleLowDepositPath extends PIDPathBuilder {
    public SampleLowDepositPath() {
        super();
        this.setPower(0.25)
                .addMappedPoint(-55.8, -55.8, 45)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(200),
                            new ArmGlobalAngleCommand(1.5),
                            new WaitCommand(100),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .waitMillis(250);
    }
}
