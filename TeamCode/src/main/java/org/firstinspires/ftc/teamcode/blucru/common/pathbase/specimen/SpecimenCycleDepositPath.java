package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleDepositPath extends PIDPathBuilder {
    public SpecimenCycleDepositPath(int scoreCount) {
        super();
        this.setPower(0.7)
                .schedule(new SequentialCommandGroup(
                        new WheelStopCommand(),
                        new ClampGrabCommand(),
                        new BoxtubeRetractCommand()
                ))
                .addMappedPoint(10 - scoreCount, -43, 270, 5)
                .schedule(
                        new SpecimenBackCommand()
                )
                .setPower(0.25)
                .addMappedPoint(7 - scoreCount * 1.5, -34.5, 270)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(500),
                        new SpecimenBackDunkCommand(),
                        new WaitCommand(100),
                        new ClampReleaseCommand(),
                        new WheelReverseCommand(),
                        new WaitCommand(100),
                        new BoxtubeRetractCommand(),
                        new WaitCommand(250),
                        new EndEffectorRetractCommand()
                ))
                .waitMillis(1800);
    }
}