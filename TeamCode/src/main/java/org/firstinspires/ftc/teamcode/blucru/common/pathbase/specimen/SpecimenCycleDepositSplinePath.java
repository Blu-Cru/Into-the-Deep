package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleDepositSplinePath extends PIDPathBuilder {
    public SpecimenCycleDepositSplinePath(int scoreCount) {
        super();
        this.setPower(0.75)
                .schedule(new SequentialCommandGroup(
                        new WheelStopCommand(),
                        new ClampGrabCommand(),
                        new BoxtubeSplineCommand(
                                new Vector2d(20,42),
                                new Pose2d(-8.6, 30, Math.PI),
                                0,
                                1.2
                        )
//                        new PivotCommand(1.2),
//                        new WaitCommand(200),
//                        new ExtensionCommand(2),
//                        new ArmGlobalAngleCommand(2)
                ))
                .addMappedPoint(10   - scoreCount, -43, 270, 5)
//                .schedule(
//                        new SpecimenBackCommand()
//                )
                .setPower(0.3)
                .addMappedPoint(7 - scoreCount * 1.5, -34, 270)
                .schedule(new SequentialCommandGroup(
                        new BoxtubeSplineCommand(
                                new Pose2d(-9.271, 21.681, Math.PI),
                                0,
                                0.3
                        ),
                        new WaitCommand(280),
                        new WheelReverseCommand(),
                        new ClampReleaseCommand(),
                        new WaitCommand(150),
                        new PivotRetractCommand(),
                        new ExtensionRetractCommand(),
                        new EndEffectorRetractCommand()
                ))
                .waitMillis(200);
    }
}