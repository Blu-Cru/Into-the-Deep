package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleIntakeFailsafePath extends PIDPathBuilder {
    public SpecimenCycleIntakeFailsafePath() {
        super();
        this.setPower(0.5)
                .schedule(
                        new SequentialCommandGroup(
                                new BoxtubeSplineCommand(
                                        new Pose2d(11, 12, 1),
                                        -Math.PI,
                                        0.4
                                ),
                                new WaitCommand(250),
                                new ClawGrabCommand()
                        )
                )
                .addMappedPoint(25, -48.5, -60)
                .waitMillis(400);
    }
}