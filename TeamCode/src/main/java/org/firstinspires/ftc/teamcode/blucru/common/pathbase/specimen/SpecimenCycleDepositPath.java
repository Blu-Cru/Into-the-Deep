package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenDunkSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleDepositPath extends PIDPathBuilder {
    public SpecimenCycleDepositPath(int scoreCount) {
        super();
        if(scoreCount == -1) {
            this.setPower(0.9)
                    .schedule(new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new BoxtubeSplineCommand(
                                    new Vector2d(20, 42),
                                    new Pose2d(-8.6, 28.5, Math.PI),
                                    0,
                                    0.95
                            )
                    ))
                    .addMappedPoint(-10, -45, 270, 30)
                    .setPower(0.7)
                    .addMappedPoint(7, -34, 270, 5)
                    .setPower(0.5)
                    .addMappedPoint(3.5, -33.5, 270, 3)
                    .schedule(new SequentialCommandGroup(
                            new WaitCommand(100),
                            new SpecimenDunkSplineCommand(),
                            new WaitCommand(320),
                            new ClawOpenCommand(),
                            new WaitCommand(70),
                            new PivotRetractCommand(),
                            new ExtensionRetractCommand(),
                            new EndEffectorRetractCommand()
                    ))
                    .waitMillis(270);
        } else {
            this.setPower(0.9)
                    .schedule(new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new BoxtubeSplineCommand(
                                    new Vector2d(20, 42),
                                    new Pose2d(-8.6, 30, Math.PI),
                                    0,
                                    0.95
                            )
                    ))
                    .addMappedPoint(10 - scoreCount * 1.2, -40, 270, 5)
                    .setPower(0.5)
                    .addMappedPoint(7 - scoreCount * 1.5, -34, 270)
                    .schedule(new SequentialCommandGroup(
                            new SpecimenDunkSplineCommand(),
                            new WaitCommand(280),
                            new ClawOpenCommand(),
                            new WaitCommand(170),
                            new PivotRetractCommand(),
                            new ExtensionRetractCommand(),
                            new EndEffectorRetractCommand()
                    ))
                    .waitMillis(170);
        }
    }

    public SpecimenCycleDepositPath() {
        this(-1);
    }
}