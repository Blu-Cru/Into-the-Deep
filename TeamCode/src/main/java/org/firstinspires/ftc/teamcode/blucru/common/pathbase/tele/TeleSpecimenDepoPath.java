package org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class TeleSpecimenDepoPath extends PIDPathBuilder {
    public TeleSpecimenDepoPath() {
        super();
        this.setPower(0.8)
                .schedule(new SequentialCommandGroup(
                        new WheelStopCommand(),
                        new ClampGrabCommand(),
                        new BoxtubeSplineCommand(
                                new Vector2d(20,42),
                                new Pose2d(-8.6, 30, Math.PI),
                                0,
                                0.95
                        )
                ))
                .addMappedPoint(0, -40, 270, 5);
    }
}
