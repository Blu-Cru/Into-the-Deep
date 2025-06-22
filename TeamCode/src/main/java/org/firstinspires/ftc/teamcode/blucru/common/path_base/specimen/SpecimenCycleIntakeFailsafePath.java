package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleIntakeFailsafePath extends PIDPathBuilder {
    public SpecimenCycleIntakeFailsafePath(int scoreCount) {
        super();
        this.setPower(0.9)
                .callback(() -> {
                    new SpecimenIntakeBackClipCommand().schedule();
                })
                .addMappedPoint(25, -52.5, 110);

        if (scoreCount < 4) {
            this.waitMillis(250);
        }
    }
}