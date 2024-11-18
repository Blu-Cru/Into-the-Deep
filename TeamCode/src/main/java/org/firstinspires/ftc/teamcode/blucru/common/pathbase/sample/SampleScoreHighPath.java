package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleScoreHighPath extends PIDPathBuilder {
    public SampleScoreHighPath() {
        super();
        this.setPower(0.2)
                .addMappedPoint(54, 54, 225)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(250),
                        new ClampReleaseCommand(),
                        new WheelReverseCommand(),
                        new WaitCommand(500),
                        new ArmGlobalAngleCommand(1.5),
                        new WaitCommand(250),
                        new WristUprightForwardCommand(),
                        new BoxtubeRetractCommand(),
                        new EndEffectorRetractCommand()
                ))
                .waitMillis(800);
    }
}
