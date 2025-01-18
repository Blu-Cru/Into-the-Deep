package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;

public class GetHooksCommand extends SequentialCommandGroup {
    public GetHooksCommand() {
        super(
                new BoxtubeSplineCommand(
                        new Pose2d(9, 2.1, -Math.PI/2),
                        -Math.PI/2,
                        0.3
                ),
                new WaitCommand(500),
                new BoxtubeSplineCommand(
//                        new Vector2d(30, -6),
                        new Pose2d(6, 2.1, -Math.PI/2),
                        -Math.PI/2,
                        0.8
                ),
                new WaitCommand(1000),
                new BoxtubeSplineCommand(
                        new Vector2d(18, -6),
                        new Pose2d(11, 1.8, -Math.PI/2),
                        -Math.PI/2,
                        0.7
                ),
                new WaitCommand(300),
                new BoxtubeSplineCommand(
                        new Pose2d(2.5, 36, 1.5),
                        -Math.PI/2,
                        3.0
                ),
                new WaitCommand(3000),
                new HangServosReleaseCommand()
        );
    }
}
