package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleIntakeSplinePath extends PIDPathBuilder {
    public SpecimenCycleIntakeSplinePath() {
        super();
        this.setPower(0.7)
                .schedule(
                        new BoxtubeSplineCommand(
                        new Pose2d(21, 10.2, 0),
                        -Math.PI,
                        1
                ))
//                .addMappedPoint(27, -48, -60, 5)
                .addMappedPoint(26, -48.5, -60)
                .schedule(new SequentialCommandGroup(
//                        new WristOppositeCommand(),
//                        new ArmGlobalAngleCommand(0),
//                        new WaitCommand(300),
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand()
                ))
                .setPower(0.2)
                .addMappedPoint(29, -53, -60)
                .waitMillis(500);
    }
}