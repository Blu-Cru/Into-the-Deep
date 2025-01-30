package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;

public class BoxtubeHooksTopBarCommand extends BoxtubeSplineCommand {
    public BoxtubeHooksTopBarCommand() {
        super(
                new Pose2d(8.2, 35.5, Math.PI/4),
                -Math.PI/2,
                0.7
        );
    }
}
