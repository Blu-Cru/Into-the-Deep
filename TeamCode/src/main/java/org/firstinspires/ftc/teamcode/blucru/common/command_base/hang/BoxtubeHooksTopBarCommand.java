package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;

public class BoxtubeHooksTopBarCommand extends BoxtubeSplineCommand {
    public BoxtubeHooksTopBarCommand() {
        super(
                new Pose2d(11, 35.5, 0.2),
                -Math.PI/2,
                0.7
        );
    }
}
