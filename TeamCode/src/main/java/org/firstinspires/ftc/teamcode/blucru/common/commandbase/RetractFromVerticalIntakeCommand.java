package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;

public class RetractFromVerticalIntakeCommand extends SequentialCommandGroup {
    public RetractFromVerticalIntakeCommand() {
        super(
                new BoxtubeSplineCommand(
                        new Vector2d(-4, 18),
                        new Pose2d(12, 5.5, 0),
                        -Math.PI/2,
                        0.35
                ),
                new WaitCommand(300),
                new FullRetractCommand()
        );
    }
}
