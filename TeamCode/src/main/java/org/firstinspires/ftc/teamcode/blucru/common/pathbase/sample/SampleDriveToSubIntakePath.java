package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;

public class SampleDriveToSubIntakePath extends PIDPathBuilder {
    public SampleDriveToSubIntakePath() {
        super();
        this.setPower(0.85)
                .addMappedPoint(-42, -12, 20, 8)
                .schedule(new BoxtubeSplineCommand(
                        new Pose2d(5, 10, -0.65),
                        -Math.PI/2,
                        0.45
                ))
                .addMappedPoint(-26, -12, 0);
    }
}
