package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;

public class BoxtubeGetHooksCommand extends SequentialCommandGroup {
    public BoxtubeGetHooksCommand() {
        super(
                new BoxtubeSplineCommand(
                        new Vector2d(18, -6),
                        new Pose2d(8.5, 1.8, -Math.PI/2),
                        -Math.PI/2,
                        0.5
                ),
                new WaitCommand(800),
                new BoxtubeSplineCommand(
                        new Vector2d(18, -6),
                        new Pose2d(12, 1.8, -Math.PI/2),
                        -Math.PI/2,
                        0.7
                ),
                new WaitCommand(300),
                new BoxtubeSplineCommand(
                        new Pose2d(2.5, 36, 1.5),
                        -Math.PI/2,
                        1.5
                )
        );
    }
}
