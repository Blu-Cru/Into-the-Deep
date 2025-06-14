package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;

public class BoxtubeHangOffGroundCommand extends BoxtubeSplineCommand {
    public BoxtubeHangOffGroundCommand() {
        super(new Pose2d(15, 20, 0.3), -Math.PI/2, 0.6);
    }
}
