package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;

public class BoxtubeHangOffGroundCommand extends BoxtubeSplineCommand {
    public BoxtubeHangOffGroundCommand() {
        super(new Pose2d(18, 18, 0), -Math.PI/2, 0.6);
    }
}
