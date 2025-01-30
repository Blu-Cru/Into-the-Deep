package org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;

public class SpecimenDunkSplineCommand extends BoxtubeSplineCommand {
    public SpecimenDunkSplineCommand() {
        super(
                new Pose2d(-9, 25.8, Math.PI),
                new Vector2d(-8, -1.5),
                new Pose2d(-9.271, 22, Math.PI),
                new Vector2d(0,0),
                0,
                0.35);
    }
}
