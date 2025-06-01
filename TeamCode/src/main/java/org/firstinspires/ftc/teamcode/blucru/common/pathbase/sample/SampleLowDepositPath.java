package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleLowDepositPath extends PIDPathBuilder {
    public SampleLowDepositPath() {
        super();
        this.setPower(0.25)
                .addMappedPoint(-55.8, -55.8, 45)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawReleaseCommand(),
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
