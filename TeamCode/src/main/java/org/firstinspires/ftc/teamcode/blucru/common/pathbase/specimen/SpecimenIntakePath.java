package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenIntakePath extends PIDPathBuilder {
    public SpecimenIntakePath(boolean firstCycle) {
        super();
        this.setPower(0.7)
                .addMappedPoint(24, -48.5, -60, 9)
                .schedule(new SequentialCommandGroup(
                        new BoxtubeSplineCommand(
                                new Pose2d(21, 10.6, 0.05),
                                -Math.PI,
                                0.5),
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand()
                ))
                .waitMillis(400)
                .setPower(0.25)
                .addMappedPoint(29, -53, -60)
                .waitMillis(150);
    }

    public SpecimenIntakePath() {
        this(false);
    }
}
