package org.firstinspires.ftc.teamcode.blucru.common.path_base.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class TeleSpecimenDepoPath extends PIDPathBuilder {
    public TeleSpecimenDepoPath() {
        super();
        this.setPower(0.8)
                .callback(() -> {
                    new SpecimenFrontFlatCommand().schedule();
                })
                .addMappedPoint(10, -40, 120, 10);
    }
}
