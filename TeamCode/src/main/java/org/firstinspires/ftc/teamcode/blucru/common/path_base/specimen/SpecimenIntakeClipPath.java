package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenIntakeClipPath extends PIDPathBuilder {
    public SpecimenIntakeClipPath(double tolerance) {
        super();
        this.setPower(0.7)
                .callback(() -> {
                    new SpecimenIntakeBackClipCommand().schedule();
                })
                .addMappedPoint(29, -54, 90, tolerance)
                .setPower(0.25)
                .addMappedPoint(36, -62, 90)
                .waitMillis(150);
    }

    public SpecimenIntakeClipPath() {
        this(5);
    }
}
