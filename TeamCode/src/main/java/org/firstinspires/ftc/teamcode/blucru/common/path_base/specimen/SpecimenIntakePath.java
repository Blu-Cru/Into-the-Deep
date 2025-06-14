package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenIntakePath extends PIDPathBuilder {
    public SpecimenIntakePath(double intakeHeight) {
        super();
        this.setPower(0.7)
                .addMappedPoint(24, -48.5, -60, 9)
                .schedule(new SequentialCommandGroup(
                        new BoxtubeSplineCommand(
                                new Pose2d(21, intakeHeight, 0.05),
                                -Math.PI,
                                0.5
                        ),
                        new ClawOpenCommand()
                ))
                .waitMillis(400)
                .setPower(0.25)
                .addMappedPoint(29, -53, -60)
                .waitMillis(150);
    }

    public SpecimenIntakePath() {
        this(11.3);
    }
}
