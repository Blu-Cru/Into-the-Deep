package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;

public class GetHooksHighCommand extends SequentialCommandGroup {
    public GetHooksHighCommand() {
        super(
                new HangServosRetractCommand(),
                new BoxtubeSplineCommand(
                        new Pose2d(9, 2.4, -Math.PI/2),
                        -Math.PI/2,
                        0.4
                ),
                new WaitCommand(500),
                new BoxtubeSplineCommand(
//                        new Vector2d(30, -6),
                        new Pose2d(5.5, 2.4, -Math.PI/2),
                        -Math.PI/2,
                        0.8
                ),
                new WaitCommand(1000),
                new BoxtubeSplineCommand(
                        new Vector2d(18, -6),
                        new Pose2d(10, 3.2, -Math.PI/2),
                        -Math.PI/2,
                        0.8
                ),
                new WaitCommand(300),
                new BoxtubeSplineCommand(
                        new Pose2d(4, 39, 0.5),
                        -Math.PI/2,
                        2.1
                ),
                new WaitCommand(1800),
                new HangServosReleaseCommand()
        );
    }
}
