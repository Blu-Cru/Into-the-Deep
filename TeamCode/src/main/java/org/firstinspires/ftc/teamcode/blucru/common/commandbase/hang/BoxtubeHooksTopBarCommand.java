package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;

public class BoxtubeHooksTopBarCommand extends BoxtubeSplineCommand {
    public BoxtubeHooksTopBarCommand() {
        super(
                new Pose2d(7, 36, 1.5),
                -Math.PI/2,
                0.7
        );
    }
}
