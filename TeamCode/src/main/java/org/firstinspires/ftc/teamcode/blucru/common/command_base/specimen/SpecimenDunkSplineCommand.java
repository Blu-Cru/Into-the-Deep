package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;

public class SpecimenDunkSplineCommand extends BoxtubeSplineCommand {
    public SpecimenDunkSplineCommand() {
        super(
                new Pose2d(-9, 28.5, Math.PI),
                new Vector2d(-5, -1.5),
                new Pose2d(-9.271, 21.5, Math.PI),
                new Vector2d(0,0),
                0,
                0.35);
    }
}
