package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleSubIntakeFailPath extends PIDPathBuilder {
    public SampleSubIntakeFailPath() {
        super();
        this.addMappedPoint(-42, -12, 0, 8)
                .addMappedPoint(-25, -12, 0);
    }
}
