package org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;

public class SampleBackHighSplineCommand extends BoxtubeSplineCommand {
    public SampleBackHighSplineCommand() {
        super(
                new Pose2d(-10.5, 46.6, 2.6),
                Math.PI/2,
                1.5
        );
    }
}
