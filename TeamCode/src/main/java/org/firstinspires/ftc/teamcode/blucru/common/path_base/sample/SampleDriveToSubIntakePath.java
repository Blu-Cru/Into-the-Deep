package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleDriveToSubIntakePath extends PIDPathBuilder {
    public SampleDriveToSubIntakePath() {
        super();
        this.setPower(0.85)
                .addMappedPoint(-42, -12, 20, 8)
                .addMappedPoint(-26, -12, 0);
    }
}
