package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenIntakeClipPath extends PIDPathBuilder {
    public SpecimenIntakeClipPath() {
        super();
        this.setPower(0.7)
                .callback(() -> {
                    new SpecimenIntakeBackClipCommand().schedule();
                })
                .addMappedPoint(29, -54, 90, 5)
                .setPower(0.25)
                .addMappedPoint(36, -62, 90)
                .waitMillis(150);
    }
}
