package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleDepositPath extends PIDPathBuilder {
    public SpecimenCycleDepositPath(int scoreCount, boolean spline) {
        super();
        if(spline) {
            this.setPower(0.8)
                    .schedule(new SequentialCommandGroup(
                            new WheelStopCommand(),
                            new ClampGrabCommand(),
                            new BoxtubeSplineCommand(
                                    new Vector2d(20,42),
                                    new Pose2d(-8.6, 30, Math.PI),
                                    0,
                                    0.95
                            )
                    ))
                    .addMappedPoint(10 - scoreCount * 1.2, -40, 270, 5)
                    .setPower(0.3)
                    .addMappedPoint(7 - scoreCount * 1.5, -34, 270)
                    .schedule(new SequentialCommandGroup(
                            new BoxtubeSplineCommand(
                                    new Pose2d(-9, 25.8, Math.PI),
                                    new Vector2d(-8, -1.5),
                                    new Pose2d(-9.271, 22, Math.PI),
                                    new Vector2d(0,0),
                                    0,
                                    0.35
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
        } else {
            this.setPower(0.7)
                    .schedule(new SequentialCommandGroup(
                            new WheelStopCommand(),
                            new ClampGrabCommand(),
                            new PivotCommand(1.2),
                            new WaitCommand(200),
                            new ExtensionCommand(2),
                            new ArmGlobalAngleCommand(2)
                    ))
                    .addMappedPoint(10   - scoreCount, -43, 270, 5)
                    .schedule(
                            new SpecimenBackCommand()
                    )
                    .setPower(0.25)
                    .addMappedPoint(7 - scoreCount * 1.5, -34, 270)
                    .schedule(new SequentialCommandGroup(
                            new WaitCommand(300),
                            new SpecimenBackDunkCommand(),
                            new WaitCommand(100),
                            new WheelReverseCommand(),
                            new ClampReleaseCommand(),
                            new WaitCommand(150),
                            new PivotRetractCommand(),
                            new ExtensionRetractCommand(),
                            new EndEffectorRetractCommand()
                    ))
                    .waitMillis(400);
        }
    }

    public SpecimenCycleDepositPath(int scoreCount) {
        this(scoreCount, true);
    }
}